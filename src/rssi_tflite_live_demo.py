import time
import json
import numpy as np
import tensorflow as tf

from live_rssi import scan_iw
from feature_vector import build_feature_vector

# ---------- SETTINGS ----------
TFLITE_PATH = "Navigation_files/nav_fp32.tflite"     
CLASSMAP_PATH = "classmap.json"       # label -> (x,y)
BSSID_ORDER_JSON = "bssid_order.json" 
INTERFACE = "wlan0"
SCAN_PERIOD_SEC = 2.0

CONF_ACCEPT = 0.80
CONF_SOFT = 0.65

# ---------- SAME NORMALIZATION AS TRAIN ----------
def normalize_rssi(vec: np.ndarray) -> np.ndarray:
    vec = np.clip(vec, -100, 0)
    return (vec + 100.0) / 100.0

def quantize(x_float: np.ndarray, scale: float, zero: int, dtype) -> np.ndarray:
    x_q = np.round(x_float / scale + zero)
    if dtype == np.int8:
        return np.clip(x_q, -128, 127).astype(np.int8)
    if dtype == np.uint8:
        return np.clip(x_q, 0, 255).astype(np.uint8)
    return x_q.astype(dtype)

def dequantize(x_q: np.ndarray, scale: float, zero: int) -> np.ndarray:
    return (x_q.astype(np.float32) - zero) * scale

def main():
    # load bssid order
    with open(BSSID_ORDER_JSON, "r") as f:
        bssid_order = json.load(f)
    print("Loaded BSSID order:", len(bssid_order))

    # load classmap
    with open(CLASSMAP_PATH, "r") as f:
        cm = json.load(f)
    class_to_xy = {int(k): (v["x"], v["y"]) for k, v in cm["cells"].items()}

    # load tflite
    interpreter = tf.lite.Interpreter(model_path=TFLITE_PATH)
    interpreter.allocate_tensors()
    in_details = interpreter.get_input_details()
    out_details = interpreter.get_output_details()

    if len(in_details) != 2:
        raise RuntimeError("Expected 2-input model (query, keys)")

    input_dim = int(in_details[0]["shape"][1])
    if input_dim != len(bssid_order):
        raise RuntimeError(f"Model expects {input_dim} features but bssid_order has {len(bssid_order)}")

    print("TFLite loaded. Input dim:", input_dim)

    while True:
        rssi_map = scan_iw(INTERFACE, sudo=True)
        vec = build_feature_vector(bssid_order, rssi_map)
        vec01 = normalize_rssi(vec)

        x = vec01.reshape(1, input_dim).astype(np.float32)

        # input0
        d0 = in_details[0]
        if d0["dtype"] in (np.int8, np.uint8):
            scale, zero = d0["quantization"]
            x0 = quantize(x, scale, zero, d0["dtype"])
        else:
            x0 = x.astype(d0["dtype"])

        # input1
        d1 = in_details[1]
        if d1["dtype"] in (np.int8, np.uint8):
            scale, zero = d1["quantization"]
            x1 = quantize(x, scale, zero, d1["dtype"])
        else:
            x1 = x.astype(d1["dtype"])

        interpreter.set_tensor(d0["index"], x0)
        interpreter.set_tensor(d1["index"], x1)
        interpreter.invoke()

        y = interpreter.get_tensor(out_details[0]["index"])[0]

        # output dequant if needed
        if out_details[0]["dtype"] in (np.int8, np.uint8):
            oscale, ozero = out_details[0]["quantization"]
            if oscale != 0:
                y = dequantize(y, oscale, ozero)

        pred_class = int(np.argmax(y))
        conf = float(np.max(y))
        xy = class_to_xy.get(pred_class, None)

        if xy is None:
            action = "REJECT_NO_MAP"
        elif conf >= CONF_ACCEPT:
            action = "ACCEPT"
        elif conf >= CONF_SOFT:
            action = "SOFT_ACCEPT"
        else:
            action = "REJECT_LOW_CONF"

        print(f"[LIVE] class={pred_class} conf={conf:.3f} action={action} xy={xy}")
        time.sleep(SCAN_PERIOD_SEC)

if __name__ == "__main__":
    main()