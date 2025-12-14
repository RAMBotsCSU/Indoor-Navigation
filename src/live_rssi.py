import re
import subprocess
from typing import Dict

BSS_RE = re.compile(r"^BSS\s+([0-9a-fA-F:]{17})", re.MULTILINE)
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
        out = subprocess.check_output(cmd, stderr=subprocess.STDOUT, timeout=timeout_sec).decode("utf-8", errors="ignore")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"iw scan failed:\n{e.output.decode('utf-8', errors='ignore')}")
    except subprocess.TimeoutExpired:
        raise RuntimeError("iw scan timed out")

    # Parse blocks: each block begins with "BSS <mac>"
    # We'll iterate line-by-line and capture current BSS + its signal.
    lines = out.splitlines()
    results = {}

    cur_bss = None
    for line in lines:
        m = re.match(r"^BSS\s+([0-9a-fA-F:]{17})", line.strip())
        if m:
            cur_bss = m.group(1).lower()
            continue

        if cur_bss is not None:
            sm = SIG_RE.search(line)
            if sm:
                results[cur_bss] = float(sm.group(1))
                # done with this BSS once signal captured
                cur_bss = None

    return results

from live_rssi import scan_iw
print(scan_iw("wlan0"))