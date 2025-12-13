import numpy as np
import math
from typing import Tuple

class RunningMap:
    def __init__(self, grid_size=200, cell_size_cm=5, max_distance_mm=6000):
        self.grid_size = grid_size
        self.cell_size_cm = cell_size_cm
        self.cell_size_mm = cell_size_cm * 10
        self.max_distance_mm = max_distance_mm

        self.map = np.zeros((grid_size, grid_size), dtype=np.float32)

        # Robot origin in grid
        self.cx = grid_size // 2
        self.cy = grid_size // 2

    def integrate_point(
        self,
        angle_deg: float,
        distance_mm: float,
        pose: Tuple[float, float, float],
    ):
        """
        Integrate a single LiDAR point into the map.

        pose: (x_cm, y_cm, theta_rad)
        """
        if distance_mm <= 0 or distance_mm > self.max_distance_mm:
            return

        x_cm, y_cm, th = pose

        # lidar point in frame
        a = math.radians(angle_deg)
        lx = distance_mm * math.cos(a)
        ly = distance_mm * math.sin(a)

        # transform to world
        wx = lx * math.cos(th) - ly * math.sin(th)
        wy = lx * math.sin(th) + ly * math.cos(th)

        # robot pos in world
        rx = x_cm * 10
        ry = y_cm * 10

        wx += rx
        wy += ry

        # convert to grid
        gx = self.cx + int(wx / self.cell_size_mm)
        gy = self.cy + int(wy / self.cell_size_mm)

        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.map[gy, gx] += 1.0

    def get_map(self, normalize=True):
        if not normalize:
            return self.map
        max_val = np.max(self.map)
        if max_val > 0:
            return self.map / max_val
        return self.map

    def reset(self):
        self.map.fill(0.0)
