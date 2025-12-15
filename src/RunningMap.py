import numpy as np
import math
import os
import matplotlib.pyplot as plt

class RunningMap:
    def __init__(self, grid_size=400, cell_size_cm=5, max_distance_mm=6000,
                 occupied_inc=0.15, free_dec=0.05, lidar_mount_offset_deg=0):
        self.grid_size = grid_size
        self.cell_size_cm = cell_size_cm
        self.max_distance_mm = max_distance_mm
        
        # Occupancy grid: 0.5 = unknown, 0 = free, 1 = occupied
        self.grid = np.full((grid_size, grid_size), 0.5, dtype=float)
        
        # Probability update parameters
        self.occupied_inc = occupied_inc  # How much to increase probability when hit
        self.free_dec = free_dec          # How much to decrease probability when free

        # LiDAR mounting offset (in radians)
        self.lidar_mount_offset_rad = math.radians(lidar_mount_offset_deg)

        # Track min/max indices to crop final overview map
        self.min_gx = grid_size // 2
        self.max_gx = grid_size // 2
        self.min_gy = grid_size // 2
        self.max_gy = grid_size // 2

    def _world_to_grid(self, wx, wy):
        """Convert world coordinates to grid coordinates."""
        cx = self.grid_size // 2
        cy = self.grid_size // 2
        gx = int(cx + wx / self.cell_size_cm)
        gy = int(cy - wy / self.cell_size_cm)
        return gx, gy

    def _is_valid(self, gx, gy):
        """Check if grid coordinates are valid."""
        return 0 <= gx < self.grid_size and 0 <= gy < self.grid_size

    def _bresenham_line(self, x0, y0, x1, y1):
        """Generate grid cells along a line using Bresenham's algorithm."""
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        return cells

    def _update_occupied(self, gx, gy):
        """Increase probability that cell is occupied."""
        if not self._is_valid(gx, gy):
            return
        
        self.grid[gy, gx] = min(1.0, self.grid[gy, gx] + self.occupied_inc)
        
        # Update bounds
        self.min_gx = min(self.min_gx, gx)
        self.max_gx = max(self.max_gx, gx)
        self.min_gy = min(self.min_gy, gy)
        self.max_gy = max(self.max_gy, gy)

    def _update_free(self, gx, gy):
        """Decrease probability that cell is occupied (i.e., it's free)."""
        if not self._is_valid(gx, gy):
            return
        
        self.grid[gy, gx] = max(0.0, self.grid[gy, gx] - self.free_dec)
        
        # Update bounds
        self.min_gx = min(self.min_gx, gx)
        self.max_gx = max(self.max_gx, gx)
        self.min_gy = min(self.min_gy, gy)
        self.max_gy = max(self.max_gy, gy)

    def integrate_point(self, angle_deg, distance_mm, pose):
        if distance_mm <= 0 or distance_mm > self.max_distance_mm:
            return

        x_robot, y_robot, th = pose
        # Add LiDAR mounting offset to angle
        angle = math.radians(angle_deg) + self.lidar_mount_offset_rad + th
        r = distance_mm / 10.0  # mm -> cm

        # Calculate endpoint (obstacle location)
        wx_end = x_robot + r * math.cos(angle)
        wy_end = y_robot + r * math.sin(angle)

        # Robot position in grid
        robot_gx, robot_gy = self._world_to_grid(x_robot, y_robot)
        end_gx, end_gy = self._world_to_grid(wx_end, wy_end)

        if not self._is_valid(robot_gx, robot_gy) or not self._is_valid(end_gx, end_gy):
            return

        # Ray trace from robot to endpoint
        ray_cells = self._bresenham_line(robot_gx, robot_gy, end_gx, end_gy)

        # Mark all cells along ray as free (except endpoint)
        for gx, gy in ray_cells[:-1]:
            self._update_free(gx, gy)

        # Mark endpoint as occupied
        self._update_occupied(end_gx, end_gy)

        #print(f"Robot world: ({x_robot:.2f}, {y_robot:.2f}), LiDAR hit world: ({wx_end:.2f}, {wy_end:.2f}), grid: ({end_gx}, {end_gy})")

    def save_heatmap(self, path, normalize=True):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        map_data = self.grid.copy()

        # Grid already in 0-1 range, no normalization needed
        plt.figure(figsize=(8, 8))
        plt.imshow(map_data, cmap="hot", origin="lower", vmin=0, vmax=1)
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

        plt.figure(figsize=(8, 8))
        plt.imshow(overview, cmap="hot", origin="lower", vmin=0, vmax=1)
        plt.axis('off')
        plt.tight_layout(pad=0)
        plt.savefig(path, bbox_inches='tight', pad_inches=0)
        plt.close()

    def save_binary_map(self, path, threshold=0.6):
        """
        Save a binary occupancy map (for navigation/planning).
        Values >= threshold are considered occupied (black).
        """
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        binary_map = (self.grid >= threshold).astype(float)
        
        plt.figure(figsize=(8, 8))
        plt.imshow(binary_map, cmap="gray_r", origin="lower", vmin=0, vmax=1)
        plt.axis('off')
        plt.tight_layout(pad=0)
        plt.savefig(path, bbox_inches='tight', pad_inches=0)
        plt.close()
