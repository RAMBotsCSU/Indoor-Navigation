import math
import json
import asyncio
import os
from typing import List, Tuple

# ---------- helpers ----------
def wrap_angle_deg(a: float) -> float:
    while a > 180:
        a -= 360
    while a < -180:
        a += 360
    return a

def dist_cm(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])

def bearing_deg(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))

# ---------- path follower ----------
class PathFollower:
    """
    Executes a cell-to-cell path using RobotController primitives.
    Pose is always read from PoseProvider, so it works for:
      - Traditional mode
      - RSSI_ML mode (ML-corrected)
    """

    def __init__(
        self,
        controller,
        pose_provider,
        classmap_path="classmap.json",
        arrive_tol_cm=25.0,
        max_turn_deg=120.0
    ):
        self.controller = controller
        self.pose_provider = pose_provider
        self.arrive_tol_cm = arrive_tol_cm
        self.max_turn_deg = max_turn_deg

        # load classmap (map-units already converted inside PoseProvider)
        BASE_DIR = os.path.dirname(os.path.abspath(__file__))
        INCLUDE_DIR = os.path.join(BASE_DIR, "..", "include")

        with open(os.path.join(INCLUDE_DIR, "classmap.json")) as f:
            _classmap = json.load(f)

        self.cell_xy_map_units = {
            int(k): (v["x"], v["y"]) for k, v in _classmap["cells"].items()
        }
       

    async def follow(self, path: List[int]):
        """
        Follow a list of cell IDs (e.g., [0,1,2,...,60])
        """
        print(f"[PathFollower] Starting path with {len(path)} cells")

        for i in range(1, len(path)):
            cur_cell = path[i - 1]
            next_cell = path[i]

            # Target in MAP UNITS → PoseProvider will align odom to this frame
            tx_map, ty_map = self.cell_xy_map_units[next_cell]

            # Convert target to CM using PoseProvider scaling
            tx_cm = tx_map * self.pose_provider.map_units_to_cm
            ty_cm = ty_map * self.pose_provider.map_units_to_cm

            # Current pose (GLOBAL frame)
            cx, cy, cth = self.pose_provider.pose()

            # Compute motion
            desired_heading = bearing_deg((cx, cy), (tx_cm, ty_cm))
            cur_heading = math.degrees(cth)
            turn_needed = wrap_angle_deg(desired_heading - cur_heading)
            distance = dist_cm((cx, cy), (tx_cm, ty_cm))

            print(
                f"[Step {i}/{len(path)-1}] "
                f"cell {cur_cell} → {next_cell} | "
                f"turn {turn_needed:.1f}°, forward {distance:.1f} cm"
            )

            # Safety: avoid insane turns
            if abs(turn_needed) > self.max_turn_deg:
                print("[WARN] Turn too large, clamping")
                turn_needed = max(-self.max_turn_deg, min(self.max_turn_deg, turn_needed))

            # Execute motion
            ok = await self.controller.turn_deg(turn_needed)
            if not ok:
                print("[ERROR] Turn failed, stopping")
                await self.controller.stop()
                return False

            ok = await self.controller.forward_cm_interpolated(distance, speed_cm_s=800.0)
            if not ok:
                print("[ERROR] Forward failed, stopping")
                await self.controller.stop()
                return False

            # Arrival check
            nx, ny, _ = self.pose_provider.pose()
            if dist_cm((nx, ny), (tx_cm, ty_cm)) <= self.arrive_tol_cm:
                print(f"[OK] Arrived at cell {next_cell}")
            else:
                print(f"[WARN] Not within tolerance of cell {next_cell}")

            await asyncio.sleep(0.1)

        print("[PathFollower] GOAL REACHED")
        return True