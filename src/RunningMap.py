import numpy as np
import math
import os
import matplotlib.pyplot as plt

class RunningMap:
    def __init__(self, grid_size=400, cell_size_cm=5, max_distance_mm=6000):
        self.grid_size = grid_size
        self.cell_size_cm = cell_size_cm
        self.max_distance_mm = max_distance_mm
        self.grid = np.zeros((grid_size, grid_size), dtype=float)

        # Track min/max indices to crop final overview map
        self.min_gx = grid_size // 2
        self.max_gx = grid_size // 2
        self.min_gy = grid_size // 2
        self.max_gy = grid_size // 2

    def integrate_point(self, angle_deg, distance_mm, pose):
        if distance_mm <= 0 or distance_mm > self.max_distance_mm:
            return

        x_robot, y_robot, th = pose
        angle = math.radians(angle_deg) + th
        r = distance_mm / 10.0  # mm -> cm

        wx = x_robot + r * math.cos(angle)
        wy = y_robot + r * math.sin(angle)

        cx = self.grid_size // 2
        cy = self.grid_size // 2

        gx = int(cx + wx / self.cell_size_cm)
        gy = int(cy - wy / self.cell_size_cm)

        # Expand grid if needed
        if gx < 0 or gx >= self.grid_size or gy < 0 or gy >= self.grid_size:
            # Skip for simplicity; alternatively implement dynamic resizing
            return

        self.grid[gy, gx] += 1.0

        # Update min/max indices for overview
        self.min_gx = min(self.min_gx, gx)
        self.max_gx = max(self.max_gx, gx)
        self.min_gy = min(self.min_gy, gy)
        self.max_gy = max(self.max_gy, gy)

    def save_heatmap(self, path, normalize=True):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        map_data = self.grid.copy()

        if normalize:
            max_val = np.max(map_data)
            if max_val > 0:
                map_data = map_data / max_val

        plt.figure(figsize=(8, 8))
        plt.imshow(map_data, cmap="hot", origin="lower")
        plt.axis('off')
        plt.tight_layout(pad=0)
        plt.savefig(path, bbox_inches='tight', pad_inches=0)
        plt.close()

    def save_overview_map(self, path, normalize=True, padding=10):
        """
        Save a cropped overview map containing all locations visited.
        """
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        min_x = max(0, self.min_gx - padding)
        max_x = min(self.grid_size, self.max_gx + padding)
        min_y = max(0, self.min_gy - padding)
        max_y = min(self.grid_size, self.max_gy + padding)

        overview = self.grid[min_y:max_y, min_x:max_x]

        if normalize:
            max_val = np.max(overview)
            if max_val > 0:
                overview = overview / max_val

        plt.figure(figsize=(8, 8))
        plt.imshow(overview, cmap="hot", origin="lower")
        plt.axis('off')
        plt.tight_layout(pad=0)
        plt.savefig(path, bbox_inches='tight', pad_inches=0)
        plt.close()
