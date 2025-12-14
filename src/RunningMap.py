import numpy as np
import math
import os
import matplotlib.pyplot as plt


class RunningMap:
    def __init__(self, grid_size=200, cell_size_cm=5, max_distance_mm=6000):
        self.grid_size = grid_size
        self.cell_size_cm = cell_size_cm
        self.cell_size_mm = cell_size_cm * 10
        self.max_distance_mm = max_distance_mm

        # Accumulated hit map
        self.overall_map = np.zeros(
            (grid_size, grid_size), dtype=np.float32
        )

    # -----------------------------------------------------

    def integrate_point(self, angle_deg, distance_mm, pose):
        """Fuse one LiDAR point into the global map."""
        if distance_mm <= 0 or distance_mm > self.max_distance_mm:
            return

        x_cm, y_cm, theta = pose

        # Convert to world coordinates (cm)
        angle_rad = math.radians(angle_deg) + theta
        dx_cm = (distance_mm / 10.0) * math.cos(angle_rad)
        dy_cm = (distance_mm / 10.0) * math.sin(angle_rad)

        wx = x_cm + dx_cm
        wy = y_cm + dy_cm

        # Convert to grid coordinates
        gx = int(self.grid_size // 2 + wx / self.cell_size_cm)
        gy = int(self.grid_size // 2 + wy / self.cell_size_cm)

        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            self.overall_map[gy, gx] += 1.0

    # -----------------------------------------------------

    def get_local_map(self, x_cm, y_cm, size_cells=120):
        """
        Return a robot-centered local window of the map.
        """
        cx = self.grid_size // 2 + int(x_cm / self.cell_size_cm)
        cy = self.grid_size // 2 + int(y_cm / self.cell_size_cm)

        half = size_cells // 2

        x0 = max(0, cx - half)
        y0 = max(0, cy - half)
        x1 = min(self.grid_size, cx + half)
        y1 = min(self.grid_size, cy + half)

        local = np.zeros((size_cells, size_cells), dtype=np.float32)

        lx0 = half - (cx - x0)
        ly0 = half - (cy - y0)

        local[
            ly0:ly0 + (y1 - y0),
            lx0:lx0 + (x1 - x0)
        ] = self.overall_map[y0:y1, x0:x1]

        return local

    # -----------------------------------------------------

    def save_heatmap(self, path, cmap="hot"):
        """Save the accumulated global map as an image."""
        os.makedirs(os.path.dirname(path), exist_ok=True)

        data = self.overall_map.copy()
        if np.max(data) > 0:
            data /= np.max(data)

        plt.figure(figsize=(8, 8))
        plt.imshow(data, cmap=cmap, origin="lower")
        plt.title("Accumulated Heatmap")
        plt.colorbar(label="Hit count (normalized)")
        plt.tight_layout()
        plt.savefig(path)
        plt.close()
