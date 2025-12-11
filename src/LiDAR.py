from adafruit_rplidar import RPLidar
from math import cos, sin, pi, floor

class LiDAR:    
    def __init__(self, port='/dev/ttyUSB0', max_distance=6000):
        self.port = port
        self.max_distance = max_distance
        self.lidar = RPLidar(None, port, timeout=3)
        self.scan_data = [0] * 360

    def get_scan(self):
        """
        Retrieve one complete scan from the LiDAR.
        Returns: list of 360 distances (one per degree 0-359).
        """
        try:
            for scan in self.lidar.iter_scans():
                # each scan is iterable of (quality, angle, distance) tuples
                scan_data = [0] * 360
                for (_, angle, distance) in scan:
                    idx = min(359, int(floor(angle)))
                    scan_data[idx] = distance
                self.scan_data = scan_data
                return scan_data
        except Exception as e:
            print(f"LiDAR error: {e}")
            return self.scan_data

    def get_scans(self, num_scans=1):
        """
        Retrieve multiple scans.
        Returns: list of lists, each inner list is 360 distances.
        """
        scans = []
        try:
            for scan in self.lidar.iter_scans():
                if len(scans) >= num_scans:
                    break
                scan_data = [0] * 360
                for (_, angle, distance) in scan:
                    idx = min(359, int(floor(angle)))
                    scan_data[idx] = distance
                scans.append(scan_data)
        except Exception as e:
            print(f"LiDAR error: {e}")
        return scans

    def get_point(self, angle_deg):
        """Get distance at a specific angle (0-359)."""
        if 0 <= angle_deg < 360:
            return self.scan_data[int(angle_deg)]
        return 0

    def to_cartesian(self, angle_deg, distance):
        """Convert polar (angle, distance) to Cartesian (x, y)."""
        radians = angle_deg * pi / 180.0
        x = distance * cos(radians)
        y = distance * sin(radians)
        return x, y

    def info(self):
        return self.lidar.get_info()

    def stop(self):
        try:
            self.lidar.stop()
            self.lidar.disconnect()
        except Exception:
            pass