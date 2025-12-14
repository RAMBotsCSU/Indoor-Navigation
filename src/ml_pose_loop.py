import os
import json
import time
import asyncio
import numpy as np
import re
import subprocess
from typing import Dict, List, Tuple

# Prefer tflite-runtime on Pi; fall back to tensorflow.lite if needed.
try:
    from tflite_runtime.interpreter import Interpreter
except ImportError:
    from tensorflow.lite.python.interpreter import Interpreter  # type: ignore


# ----------------------------
# Paths (JSON lives in ../include)
# ----------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
INCLUDE_DIR = os.path.join(BASE_DIR, "..", "include")
MODELS_DIR  = os.path.join(BASE_DIR, "..", "Navigation_files")

BSSID_ORDER_PATH = os.path.join(INCLUDE_DIR, "bssid_order.json")
CLASSMAP_PATH     = os.path.join(INCLUDE_DIR, "classmap.json")

# If your model is stored elsewhere, change this path.
# Common choices:
#   ../include/model.tflite
#   ../models/model.tflite
DEFAULT_MODEL_PATH = os.path.join(MODELS_DIR, "nav_fp32.tflite")


# ----------------------------
# iw scan parsing (BSSID + RSSI)
# ----------------------------
SIG_RE = re.compile(r"signal:\s+(-?\d+(?:\.\d+)?)\s+dBm")

def scan_iw(interface: str = "wlan0", sudo: bool = True, timeout_sec: int = 8) -> Dict[str, float]:
    """
    Returns dict: {bssid_lower: rssi_dbm}
    Uses: iw dev <iface> scan
    """
    cmd = ["iw", "dev", interface, "scan"]
    if sudo:
        cmd = ["sudo"] + cmd

    try:
        out = subprocess.check_output(
            cmd,
            stderr=subprocess.STDOUT,
            timeout=timeout_sec
        ).decode("utf-8", errors="ignore")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"iw scan failed:\n{e.output.decode('utf-8', errors='ignore')}")
    except subprocess.TimeoutExpired:
        raise RuntimeError("iw scan timed out")

    lines = out.splitlines()
    results: Dict[str, float] = {}

    cur_bss = None
    for line in lines:
        line_stripped = line.strip()
        if line_stripped.startswith("BSS "):
            parts = line_stripped.split()
            if len(parts) >= 2:
                cur_bss = parts[1].lower()
            else:
                cur_bss = None
            continue

        if cur_bss is not None:
            sm = SIG_RE.search(line_stripped)
            if sm:
                results[cur_bss] = float(sm.group(1))
                cur_bss = None

    return results


# ----------------------------
# JSON loaders
# ----------------------------
def load_bssid_order(path: str = BSSID_ORDER_PATH) -> List[str]:
    with open(path, "r") as f:
        data = json.load(f)
    # Accept either list or {"bssid_order":[...]} formats
    if isinstance(data, dict) and "bssid_order" in data:
        order = data["bssid_order"]
    else:
        order = data
    return [str(x).lower() for x in order]

def load_classmap(path: str = CLASSMAP_PATH) -> Dict[int, Tuple[float, float]]:
    """
    Returns {class_id: (x_map, y_map)} where x_map,y_map are in 'map units' (meters/cells).
    """
    with open(path, "r") as f:
        cm = json.load(f)

    # Expect structure like {"cells": {"0":{"x":..,"y":..}, ...}}
    cells = cm.get("cells", cm)
    out: Dict[int, Tuple[float, float]] = {}

    for k, v in cells.items():
        cid = int(k)
        out[cid] = (float(v["x"]), float(v["y"]))
    return out


# ----------------------------
# Feature build + (optional) normalization
# ----------------------------
def build_feature_vector(
    scan: Dict[str, float],
    bssid_order: List[str],
    missing_dbm: float = -100.0
) -> np.ndarray:
    """
    Build [1, N] vector in BSSID order. Missing APs -> missing_dbm.
    """
    feats = np.full((len(bssid_order),), float(missing_dbm), dtype=np.float32)
    for i, bssid in enumerate(bssid_order):
        if bssid in scan:
            feats[i] = float(scan[bssid])
    return feats.reshape(1, -1)  # [1, N]

def maybe_normalize(x: np.ndarray) -> np.ndarray:
    """
    If your model expects normalized input (e.g., scaled RSSI),
    apply the SAME transform you used in training.

    Right now, this is identity.
    """
    return x


# ----------------------------
# Softmax + confidence
# ----------------------------
def softmax(logits: np.ndarray) -> np.ndarray:
    z = logits - np.max(logits)
    exp = np.exp(z)
    return exp / np.sum(exp)

def extract_prediction(output: np.ndarray) -> Tuple[int, float]:
    """
    Returns (pred_class, conf).
    Handles either:
      - probabilities already
      - logits (we softmax)
    """
    out = np.squeeze(output).astype(np.float32)

    # Heuristic: if values don't look like probabilities, softmax them
    if np.any(out < 0.0) or np.any(out > 1.0) or not np.isclose(np.sum(out), 1.0, atol=0.05):
        probs = softmax(out)
    else:
        probs = out

    pred = int(np.argmax(probs))
    conf = float(np.max(probs))
    return pred, conf


# ----------------------------
# Main ML pose loop
# ----------------------------
async def ml_pose_update_loop(
    pose_provider,
    model_path: str = DEFAULT_MODEL_PATH,
    interface: str = "wlan0",
    scan_hz: float = 1.0,
    sudo_scan: bool = True,
    missing_dbm: float = -100.0
):
    """
    Loop:
      iw scan -> build [1,193] -> tflite -> class -> (x_map,y_map) -> update_ml_map_units()

    Requirements:
      - include/bssid_order.json
      - include/classmap.json
      - model_path points to a .tflite file
    """
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"TFLite model not found: {model_path}")

    bssid_order = load_bssid_order(BSSID_ORDER_PATH)
    class_xy = load_classmap(CLASSMAP_PATH)

    # init interpreter
    interpreter = Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    in_idx = input_details[0]["index"]
    out_idx = output_details[0]["index"]

    period = 1.0 / max(scan_hz, 0.1)

    print(f"[ML] Loaded model: {model_path}")
    print(f"[ML] BSSIDs in order: {len(bssid_order)}")
    print(f"[ML] Classmap size: {len(class_xy)}")
    print(f"[ML] Scanning {interface} at ~{scan_hz:.2f} Hz")

    while True:
        t0 = time.time()

        try:
            scan = await asyncio.to_thread(scan_iw, interface, sudo_scan, 8)
        except Exception as e:
            print(f"[ML] scan error: {e}")
            await asyncio.sleep(period)
            continue

        x = build_feature_vector(scan, bssid_order, missing_dbm=missing_dbm)
        x = maybe_normalize(x)

        # Ensure dtype matches model
        x = x.astype(np.float32)

        interpreter.set_tensor(in_idx, x)
        interpreter.invoke()
        y = interpreter.get_tensor(out_idx)

        pred_class, conf = extract_prediction(y)

        if pred_class not in class_xy:
            print(f"[ML] predicted class {pred_class} not in classmap")
        else:
            x_map, y_map = class_xy[pred_class]
            # push into PoseProvider (map-units)
            pose_provider.update_ml_map_units(x_map, y_map, conf)

            # Optional debug print (rate-limited a bit)
            print(f"[ML] class={pred_class} conf={conf:.2f} -> (x_map={x_map:.2f}, y_map={y_map:.2f})")

        dt = time.time() - t0
        if dt < period:
            await asyncio.sleep(period - dt)