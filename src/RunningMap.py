from math import cos, sin, pi, floor
import numpy as np

GRID_RESOLUTION = 1024

class RunningMap:
    def __init__(self, grid_size=200, cell_size_cm=5, max_distance_mm=6000):
        """
        grid_size: number of cells per axis (grid_size x grid_size)
        cell_size_cm: centimeters per cell
        max_distance_mm: clamp distances to this value
        """
        self.grid_size = grid_size
        self.cell_size_cm = cell_size_cm
        self.cell_size_mm = cell_size_cm * 10  # convert to mm for internal use
        self.max_distance_mm = max_distance_mm
        self.frames = []  # list of individual heatmap frames
        self.overall_map = np.zeros((self.grid_size, self.grid_size), dtype=float)  # accumulated map

    def heatmap_from_scan(self, scan_data, position=(0.0, 0.0)):
        """
        Convert LiDAR scan data and robot position to a heatmap frame.
        Saves frame and updates the overall accumulated map.
        
        scan_data: list of 360 distances (one per degree 0-359) in mm
        position: tuple (x_cm, y_cm) of robot center in cm
        Returns: 2D numpy array (heatmap grid for this scan)
        """
        # ensure scan_data has 360 entries
        if len(scan_data) < 360:
            scan_data = list(scan_data) + [0.0] * (360 - len(scan_data))
        elif len(scan_data) > 360:
            scan_data = scan_data[:360]

        # clamp distances
        distances = [max(0.0, min(self.max_distance_mm, float(d))) for d in scan_data]

        # build the frame
        frame = self._frame_from_scan(distances, position=position)
        
        # save frame and accumulate into overall map
        self.frames.append(frame)
        self.overall_map += frame
        
        return frame

    def _frame_from_scan(self, distances, position=None):
        """Convert one 360-distance scan into a grid heatmap.
        
        position: tuple (x_cm, y_cm) of robot center in cm; defaults to (0, 0)
        """
        if position is None:
            position = (0.0, 0.0)

        grid = np.zeros((self.grid_size, self.grid_size), dtype=float)
        
        # convert robot position (cm) to grid coords
        rx_cm, ry_cm = position
        rx = self.grid_size // 2 + int(rx_cm / self.cell_size_cm)
        ry = self.grid_size // 2 + int(ry_cm / self.cell_size_cm)

        # add scan points relative to robot position
        for ang in range(360):
            d = distances[ang]
            if d <= 0:
                continue
            x, y = self.angular_to_cartesian(ang, d)
            gx = rx + int(x / self.cell_size_mm)
            gy = ry + int(y / self.cell_size_mm)
            if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                grid[gy, gx] += 1.0

        return grid

    def get_overall_map(self):
        """Get the accumulated overall map (all scans combined)."""
        return self.overall_map

    def get_frame(self, idx):
        """Get a single frame by index."""
        if idx < len(self.frames):
            return self.frames[idx]
        return None

    def num_frames(self):
        """Get the number of frames captured."""
        return len(self.frames)

    def reset(self):
        """Clear all frames and reset the overall map."""
        self.frames = []
        self.overall_map = np.zeros((self.grid_size, self.grid_size), dtype=float)

    def save_overall_map(self, out_path, cmap="hot", normalize=True):
        """Save the overall accumulated map as an image."""
        import matplotlib.pyplot as plt
        
        map_data = self.overall_map
        if normalize:
            # normalize to 0-1 for better visualization
            max_val = np.max(map_data)
            if max_val > 0:
                map_data = map_data / max_val

        fig, ax = plt.subplots(figsize=(8, 8))
        im = ax.imshow(map_data, cmap=cmap, origin="lower")
        ax.set_title("Overall Accumulated Map")
        plt.colorbar(im, ax=ax, label="accumulated hits")
        plt.tight_layout()
        plt.savefig(out_path)
        plt.close()

    def display_overall_map(self, cmap="hot", normalize=True):
        """Display the overall accumulated map."""
        import matplotlib.pyplot as plt
        
        map_data = self.overall_map
        if normalize:
            max_val = np.max(map_data)
            if max_val > 0:
                map_data = map_data / max_val

        fig, ax = plt.subplots(figsize=(8, 8))
        im = ax.imshow(map_data, cmap=cmap, origin="lower")
        ax.set_title("Overall Accumulated Map")
        plt.colorbar(im, ax=ax, label="accumulated hits")
        plt.tight_layout()
        plt.show()

    def angular_to_cartesian(self, angle_deg, distance):
        """Convert polar (angle, distance) to Cartesian (x, y)."""
        radians = angle_deg * pi / 180.0
        x = distance * cos(radians)
        y = distance * sin(radians)
        return x, y