import os
import json
import time
import asyncio
import numpy as np
import re
import subprocess
import math
import heapq
from typing import Dict, List, Tuple

try:
    from tflite_runtime.interpreter import Interpreter, load_delegate
except ImportError:
    from tensorflow.lite.python.interpreter import Interpreter  # type: ignore
    load_delegate = None

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
INCLUDE_DIR = os.path.join(BASE_DIR, "..", "include")
MODELS_DIR = os.path.join(BASE_DIR, "..", "Navigation_files")

BSSID_ORDER_PATH = os.path.join(INCLUDE_DIR, "bssid_order.json")
CLASSMAP_PATH = os.path.join(INCLUDE_DIR, "classmap.json")
NEIGHBORS_PATH = os.path.join(INCLUDE_DIR, "neighbors.json")
DEFAULT_MODEL_PATH = os.path.join(MODELS_DIR, "nav_fp32.tflite")

SIG_RE = re.compile(r"signal:\s+(-?\d+(?:\.\d+)?)\s+dBm")


class NavToolkit:
    """Unified helpers for RSSI scanning, feature vectors, inference, and A* planning."""
    _bssid_cache: List[str] = []
    _class_xy_cache: Dict[int, Tuple[float, float]] = {}
    _class_pos_cache: Dict[int, Tuple[float, float]] = {}
    _neighbors_cache: Dict[int, List[int]] = {}

    # ---------- Wi-Fi scan ----------
    @classmethod
    def scan_iw(cls, interface: str = "wlan0", sudo: bool = True, timeout_sec: int = 8) -> Dict[str, float]:
        cmd = ["iw", "dev", interface, "scan"]
        if sudo:
            cmd = ["sudo"] + cmd
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=timeout_sec).decode("utf-8", errors="ignore")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"iw scan failed:\n{e.output.decode('utf-8', errors='ignore')}")
        except subprocess.TimeoutExpired:
            raise RuntimeError("iw scan timed out")

        results: Dict[str, float] = {}
        cur_bss = None
        for line in out.splitlines():
            ls = line.strip()
            if ls.startswith("BSS "):
                parts = ls.split()
                cur_bss = parts[1].lower() if len(parts) >= 2 else None
                continue
            if cur_bss:
                sm = SIG_RE.search(ls)
                if sm:
                    results[cur_bss] = float(sm.group(1))
                    cur_bss = None
        return results

    # ---------- JSON loaders ----------
    @classmethod
    def load_bssid_order(cls, path: str = BSSID_ORDER_PATH) -> List[str]:
        if cls._bssid_cache:
            return cls._bssid_cache
        with open(path, "r") as f:
            data = json.load(f)
        order = data["bssid_order"] if isinstance(data, dict) and "bssid_order" in data else data
        cls._bssid_cache = [str(x).lower() for x in order]
        return cls._bssid_cache

    @classmethod
    def load_classmap(cls, path: str = CLASSMAP_PATH) -> Dict[int, Tuple[float, float]]:
        if cls._class_xy_cache:
            return cls._class_xy_cache
        with open(path, "r") as f:
            cm = json.load(f)
        cells = cm.get("cells", cm)
        cls._class_xy_cache = {int(k): (float(v["x"]), float(v["y"])) for k, v in cells.items()}
        return cls._class_xy_cache

    @classmethod
    def _load_graph(cls):
        if cls._class_pos_cache and cls._neighbors_cache:
            return
        with open(CLASSMAP_PATH, "r") as f:
            _classmap = json.load(f)["cells"]
        with open(NEIGHBORS_PATH, "r") as f:
            _neighbors = json.load(f)["neighbors"]
        cls._class_pos_cache = {int(k): (v["x"], v["y"]) for k, v in _classmap.items()}
        cls._neighbors_cache = {int(k): v for k, v in _neighbors.items()}

    # ---------- Features ----------
    @staticmethod
    def build_feature_vector(scan: Dict[str, float], bssid_order: List[str], missing_dbm: float = -100.0) -> np.ndarray:
        feats = np.full((len(bssid_order),), float(missing_dbm), dtype=np.float32)
        for i, bssid in enumerate(bssid_order):
            if bssid in scan:
                feats[i] = float(scan[bssid])
        return feats.reshape(1, -1)

    @staticmethod
    def maybe_normalize(x: np.ndarray) -> np.ndarray:
        return x

    # ---------- Inference helpers ----------
    @staticmethod
    def softmax(logits: np.ndarray) -> np.ndarray:
        z = logits - np.max(logits)
        exp = np.exp(z)
        return exp / np.sum(exp)

    @classmethod
    def extract_prediction(cls, output: np.ndarray) -> Tuple[int, float]:
        out = np.squeeze(output).astype(np.float32)
        probs = cls.softmax(out) if (np.any(out < 0.0) or np.any(out > 1.0) or not np.isclose(np.sum(out), 1.0, atol=0.05)) else out
        pred = int(np.argmax(probs))
        conf = float(np.max(probs))
        return pred, conf

    # ---------- A* ----------
    @classmethod
    def _heuristic(cls, a: int, b: int) -> float:
        ax, ay = cls._class_pos_cache[a]
        bx, by = cls._class_pos_cache[b]
        return math.hypot(bx - ax, by - ay)

    @classmethod
    def astar(cls, start: int, goal: int):
        cls._load_graph()
        if start == goal:
            return [start]
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0.0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal:
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            for nb in cls._neighbors_cache.get(current, []):
                tentative = g_score[current] + cls._heuristic(current, nb)
                if nb not in g_score or tentative < g_score[nb]:
                    came_from[nb] = current
                    g_score[nb] = tentative
                    f = tentative + cls._heuristic(nb, goal)
                    heapq.heappush(open_set, (f, nb))
        return None

    # ---------- ML loop ----------
    @classmethod
    def _make_interpreter(cls, model_path: str):
        """Create interpreter with Coral delegate if available and model is EdgeTPU-compiled."""
        if load_delegate and model_path.endswith(".edgetpu.tflite"):
            try:
                print("[ML] Trying Coral Edge TPU delegate")
                return Interpreter(
                    model_path=model_path,
                    experimental_delegates=[load_delegate("libedgetpu.so.1")]
                )
            except Exception as e:
                print(f"[ML] TPU delegate failed, falling back to CPU: {e}")
        return Interpreter(model_path=model_path)

    @classmethod
    async def ml_pose_update_loop(
        cls,
        pose_provider,
        model_path: str = DEFAULT_MODEL_PATH,
        interface: str = "wlan0",
        scan_hz: float = 1.0,
        sudo_scan: bool = True,
        missing_dbm: float = -100.0
    ):
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"TFLite model not found: {model_path}")

        bssid_order = cls.load_bssid_order(BSSID_ORDER_PATH)
        class_xy = cls.load_classmap(CLASSMAP_PATH)

        interpreter = cls._make_interpreter(model_path)
        interpreter.allocate_tensors()
        in_idx = interpreter.get_input_details()[0]["index"]
        out_idx = interpreter.get_output_details()[0]["index"]

        period = 1.0 / max(scan_hz, 0.1)

        print(f"[ML] Loaded model: {model_path}")
        print(f"[ML] BSSIDs in order: {len(bssid_order)}")
        print(f"[ML] Classmap size: {len(class_xy)}")
        print(f"[ML] Scanning {interface} at ~{scan_hz:.2f} Hz")

        while True:
            t0 = time.time()
            try:
                scan = await asyncio.to_thread(cls.scan_iw, interface, sudo_scan, 8)
            except Exception as e:
                print(f"[ML] scan error: {e}")
                await asyncio.sleep(period)
                continue

            x = cls.build_feature_vector(scan, bssid_order, missing_dbm=missing_dbm)
            x = cls.maybe_normalize(x).astype(np.float32)

            interpreter.set_tensor(in_idx, x)
            interpreter.invoke()
            y = interpreter.get_tensor(out_idx)

            pred_class, conf = cls.extract_prediction(y)
            if pred_class in class_xy:
                x_map, y_map = class_xy[pred_class]
                pose_provider.update_ml_map_units(x_map, y_map, conf)
                print(f"[ML] class={pred_class} conf={conf:.2f} -> (x_map={x_map:.2f}, y_map={y_map:.2f})")
            else:
                print(f"[ML] predicted class {pred_class} not in classmap")

            dt = time.time() - t0
            if dt < period:
                await asyncio.sleep(period - dt)
