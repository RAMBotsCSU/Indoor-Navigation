from math import cos, sin, pi, floor
import numpy as np
GRID_RESOLUTION = 1024

class RunningMap:
    def __init__(self, grid_size=200, cell_size_mm=50, max_distance_mm=6000):
        """
        grid_size: number of cells per axis (grid_size x grid_size)
        cell_size_mm: millimeters per cell
        max_distance_mm: clamp distances to this value
        """
        self.grid_size = grid_size
        self.cell_size_mm = cell_size_mm
        self.max_distance_mm = max_distance_mm
        self.frames = []  # list of np.ndarray heatmaps

    def heatmap_from_csv(self, filename, timesteps=None):
        """
        Build heatmap frames from CSV. Each row = timestep; cols 0..359 = distance in mm.
        timesteps: optional limit on number of rows to read.
        """
        self.frames = []
        with open(filename, "r") as f:
            for t, line in enumerate(f):
                if timesteps is not None and t >= timesteps:
                    break
                parts = [p.strip() for p in line.strip().split(",") if p.strip() != ""]
                if not parts:
                    continue
                # clamp/convert distances
                distances = []
                for i in range(min(len(parts), 360)):
                    try:
                        d = float(parts[i])
                    except Exception:
                        d = 0.0
                    distances.append(max(0.0, min(self.max_distance_mm, d)))
                # pad if fewer than 360
                if len(distances) < 360:
                    distances += [0.0] * (360 - len(distances))
                frame = self._frame_from_scan(distances)
                self.frames.append(frame)
        return self.frames

    def _frame_from_scan(self, distances):
        """Convert one 360-distance scan into a grid heatmap."""
        grid = np.zeros((self.grid_size, self.grid_size), dtype=float)
        cx = cy = self.grid_size // 2  # robot at center
        for ang in range(360):
            d = distances[ang]
            if d <= 0:
                continue
            x, y = self.angular_to_cartesian(ang, d)
            gx = cx + int(x / self.cell_size_mm)
            gy = cy + int(y / self.cell_size_mm)
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                grid[gy, gx] += 1.0
        return grid

    def frame_at(self, idx):
        return self.frames[idx]

    def save_frame(self, idx, out_path, cmap="hot"):
        """ Save heatmap frame as image """
        import matplotlib.pyplot as plt
        plt.imshow(self.frame_at(idx), cmap=cmap, origin="lower")
        plt.title(f"t={idx}")
        plt.colorbar(label="hits")
        plt.tight_layout()
        plt.savefig(out_path)
        plt.close()

    # function to granularize lidar points into grid points
    def normalize_point(self, radius, angle):
        x, y = self.angular_to_cartesian(angle, radius)
        grid_x = floor(x / GRID_RESOLUTION)
        grid_y = floor(y / GRID_RESOLUTION)
        return grid_x, grid_y

    def angular_to_cartesian(self, angle_deg, distance):
        radians = angle_deg * pi / 180.0
        x = distance * cos(radians)
        y = distance * sin(radians)
        return x, y