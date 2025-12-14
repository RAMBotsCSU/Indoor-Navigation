import numpy as np
from typing import Dict, List

DEFAULT_RSSI = -100.0

def build_feature_vector(bssid_order: List[str], rssi_map: Dict[str, float]) -> np.ndarray:
    """
    bssid_order: list of 193 BSSID strings in the exact CSV header order
    rssi_map: {bssid_lower: rssi_dbm}
    Returns: np.ndarray shape (193,) float32
    """
    vec = np.full((len(bssid_order),), DEFAULT_RSSI, dtype=np.float32)
    for i, b in enumerate(bssid_order):
        key = b.lower()
        if key in rssi_map:
            vec[i] = float(rssi_map[key])
    return vec