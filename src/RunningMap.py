import numpy as np
import math
import os
import matplotlib.pyplot as plt

class RunningMap:
    def __init__(self, grid_size=200, cell_size_cm=5, max_distance_mm=6000):
        self.grid_size = grid_size
        self.cell_size_cm = cell_size_cm
        self.max_distance_mm = max_distance_mm
        self.grid = np.zeros((grid_size, grid_size), dtype=float)

    def integrate_point(self, angle_deg, distance_mm, pose):
        """Add a LiDAR point to the map using world coordinates."""
        x_robot, y_robot, th = pose
        angle = math.radians(angle_deg) + th
        r = distance_mm / 10.0  # mm -> cm

        wx = x_robot + r * math.cos(angle)
        wy = y_robot + r * math.sin(angle)

        # Center robot in map
        cx = self.grid_size // 2
        cy = self.grid_size // 2

        gx = int(cx + wx / self.cell_size_cm)
        gy = int(cy - wy / self.cell_size_cm)  # flip Y-axis

        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.grid[gy, gx] += 1.0

    def save_heatmap(self, path, cmap="hot", normalize=True):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        map_data = self.grid.copy()
        if normalize:
            max_val = np.max(map_data)
            if max_val > 0:
                map_data = map_data / max_val

        plt.figure(figsize=(8, 8))
        plt.imshow(map_data, cmap=cmap, origin="lower")
        plt.title("Accumulated Map")
        plt.colorbar(label="hits")
        plt.tight_layout()
        plt.savefig(path)
        plt.close()
