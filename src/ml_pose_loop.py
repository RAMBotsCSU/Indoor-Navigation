import asyncio
import json
import numpy as np
import tensorflow as tf

from live_rssi import scan_iw
from feature_vector import build_feature_vector

def normalize_rssi(vec: np.ndarray) -> np.ndarray:
    vec = np.clip(vec, -100, 0)
    return (vec + 100.0) / 100.0

async def ml_pose_update_loop(
    pose_provider,
    tflite_path: str,
    classmap_path: str,
    bssid_order_json: str,
    interface: str = "wlan0",
    period_s: float = 2.0,
):
    # Load BSSID order
    with open(bssid_order_json, "r") as f:
        bssid_order = json.load(f)

    # Load classmap (map units)
    with open(classmap_path, "r") as f:
        cm = json.load(f)
    class_to_xy = {int(k): (v["x"], v["y"]) for k, v in cm["cells"].items()}

    # Load TFLite
    interpreter = tf.lite.Interpreter(model_path=tflite_path)
    interpreter.allocate_tensors()
    in_details = interpreter.get_input_details()
    out_details = interpreter.get_output_details()

    if len(in_details) != 2:
        raise RuntimeError("Expected 2-input model (query, keys)")

    input_dim = int(in_details[0]["shape"][1])
    if input_dim != len(bssid_order):
        raise RuntimeError(f"Model expects {input_dim} features but bssid_order has {len(bssid_order)}")

    while True:
        rssi_map = scan_iw(interface, sudo=True)
        vec = build_feature_vector(bssid_order, rssi_map)        # raw dBm
        vec01 = normalize_rssi(vec).reshape(1, input_dim).astype(np.float32)

        # feed both inputs (works for fp32/dynamic; int8 needs quantize handling)
        interpreter.set_tensor(in_details[0]["index"], vec01.astype(in_details[0]["dtype"]))
        interpreter.set_tensor(in_details[1]["index"], vec01.astype(in_details[1]["dtype"]))
        interpreter.invoke()

        y = interpreter.get_tensor(out_details[0]["index"])[0].astype(np.float32)
        cls = int(np.argmax(y))
        conf = float(np.max(y))

        xy = class_to_xy.get(cls, None)
        if xy is not None:
            pose_provider.update_ml_map_units(xy[0], xy[1], conf)

        await asyncio.sleep(period_s)