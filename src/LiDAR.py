from adafruit_rplidar import RPLidar
from math import cos, sin, pi, floor
import time
import serial

class LiDAR:    
    def __init__(self, port='/dev/ttyUSB0', max_distance=6000, max_retries=3):
        self.port = port
        self.max_distance = max_distance
        self.max_retries = max_retries
        self.scan_data = [0] * 360
        self.lidar = None
        self._initialize()

    def _initialize(self):
        """Initialize LiDAR with retry logic to handle descriptor errors."""
        for attempt in range(self.max_retries):
            try:
                # Clean up any existing connection
                if self.lidar is not None:
                    try:
                        self.lidar.stop()
                        self.lidar.disconnect()
                    except Exception:
                        pass
                    self.lidar = None
                
                # Close any lingering serial connections
                try:
                    ser = serial.Serial(self.port)
                    ser.close()
                    time.sleep(0.5)
                except Exception:
                    pass
                
                # Initialize with shorter timeout initially
                self.lidar = RPLidar(None, self.port, timeout=1)
                time.sleep(1)
                
                # Stop any running motor/scan first
                try:
                    self.lidar.stop()
                    self.lidar.stop_motor()
                except Exception:
                    pass
                time.sleep(0.5)
                
                # Try to get device info if available (not critical)
                try:
                    if hasattr(self.lidar, 'get_info'):
                        info = self.lidar.get_info()
                        print(f"LiDAR connected: {info}")
                    elif hasattr(self.lidar, 'info'):
                        info = self.lidar.info
                        print(f"LiDAR connected: {info}")
                except Exception as e:
                    print(f"Could not get device info (non-critical): {e}")
                
                # Start motor
                self.lidar.start_motor()
                time.sleep(1)
                
                # Verify with quick test scan
                test_scan = None
                for scan in self.lidar.iter_scans():
                    test_scan = scan
                    break
                
                if test_scan is not None:
                    print(f"LiDAR initialized successfully on attempt {attempt + 1}")
                    return
                    
            except Exception as e:
                print(f"LiDAR init attempt {attempt + 1} failed: {e}")
                if self.lidar:
                    try:
                        self.lidar.stop()
                        self.lidar.disconnect()
                    except Exception:
                        pass
                    self.lidar = None
                time.sleep(2)
        
        raise Exception("Failed to initialize LiDAR after maximum retries")

    def get_scan(self):
        """
        Retrieve one complete scan from the LiDAR.
        Returns: list of 360 distances (one per degree 0-359).
        """
        try:
            for scan in self.lidar.iter_scans():
                scan_data = [0] * 360
                for (_, angle, distance) in scan:
                    if distance > 0 and distance < self.max_distance:
                        idx = min(359, int(floor(angle)))
                        scan_data[idx] = distance
                self.scan_data = scan_data
                return scan_data
        except Exception as e:
            print(f"LiDAR scan error: {e}")
            try:
                self._initialize()
            except Exception as recovery_error:
                print(f"LiDAR recovery failed: {recovery_error}")
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
                    if distance > 0 and distance < self.max_distance:
                        idx = min(359, int(floor(angle)))
                        scan_data[idx] = distance
                scans.append(scan_data)
        except Exception as e:
            print(f"LiDAR error: {e}")
            try:
                self._initialize()
            except Exception as recovery_error:
                print(f"LiDAR recovery failed: {recovery_error}")
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
        """Get device info if available."""
        try:
            if hasattr(self.lidar, 'get_info'):
                return self.lidar.get_info()
            elif hasattr(self.lidar, 'info'):
                return self.lidar.info
            else:
                return "Info method not available"
        except Exception as e:
            return f"Error getting info: {e}"

    def stop(self):
        """Stop the LiDAR and clean up."""
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except Exception as e:
            print(f"Error during stop: {e}")