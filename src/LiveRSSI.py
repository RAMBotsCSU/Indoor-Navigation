import re
import subprocess
from typing import Dict

BSS_RE = re.compile(r"^BSS\s+([0-9a-fA-F:]{17})", re.MULTILINE)
SIG_RE = re.compile(r"signal:\s+(-?\d+(?:\.\d+)?)\s+dBm")

class LiveRSSI:
    def __init__(self, interface: str = "wlan0", sudo: bool = True, timeout_sec: int = 8):
        """Use interface='wlan0' on Coral; set sudo=False if already running as root."""
        self.interface = interface
        self.sudo = sudo
        self.timeout_sec = timeout_sec

    def scan(self) -> Dict[str, float]:
        """Returns dict {bssid_lower: rssi_dbm} using `iw dev <iface> scan`."""
        cmd = ["iw", "dev", self.interface, "scan"]
        if self.sudo:
            cmd = ["sudo"] + cmd
        out = self._run_cmd(cmd)
        return self._parse_scan(out)

    def _run_cmd(self, cmd) -> str:
        try:
            return subprocess.check_output(
                cmd, stderr=subprocess.STDOUT, timeout=self.timeout_sec
            ).decode("utf-8", errors="ignore")
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"iw scan failed:\n{e.output.decode('utf-8', errors='ignore')}")
        except subprocess.TimeoutExpired:
            raise RuntimeError("iw scan timed out")

    @staticmethod
    def _parse_scan(out: str) -> Dict[str, float]:
        lines = out.splitlines()
        results = {}
        cur_bss = None
        for line in lines:
            m = BSS_RE.match(line.strip())
            if m:
                cur_bss = m.group(1).lower()
                continue
            if cur_bss is not None:
                sm = SIG_RE.search(line)
                if sm:
                    results[cur_bss] = float(sm.group(1))
                    cur_bss = None
        return results