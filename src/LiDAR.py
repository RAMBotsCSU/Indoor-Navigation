from adafruit_rplidar import RPLidar
from math import cos, sin, pi, floor
import asyncio

class LiDAR:    
    def __init__(self, port='/dev/ttyUSB0', max_distance=6000):
        self.port = port
        self.max_distance = max_distance
        self.scan_data = [0] * 360
        self.lidar = None

    async def connect(self):
        """Async initialize connection with reset/clear."""
        self.lidar = await asyncio.to_thread(RPLidar, None, self.port, 3)
        try:
            await asyncio.to_thread(self.lidar.stop)
            await asyncio.to_thread(self.lidar.stop_motor)
        except Exception:
            pass
        await asyncio.sleep(0.5)
        try:
            await asyncio.to_thread(self.lidar.clear_input)
        except Exception:
            pass
        await asyncio.sleep(0.2)
        try:
            await asyncio.to_thread(self.lidar.start_motor)
        except Exception:
            pass
        await asyncio.sleep(1)

    async def get_scan(self):
        """
        Retrieve one complete scan from the LiDAR asynchronously.
        Returns: list of 360 distances (one per degree 0-359).
        """
        if not self.lidar:
            print("LiDAR not connected")
            return self.scan_data
        
        async def _one_scan():
            for scan in self.lidar.iter_scans():
                scan_data = [0] * 360
                for (_, angle, distance) in scan:
                    idx = min(359, int(floor(angle)))
                    scan_data[idx] = distance
                return scan_data
            return self.scan_data

        retries = 3
        for attempt in range(retries):
            try:
                data = await asyncio.to_thread(_one_scan)
                if data:
                    self.scan_data = data
                    return data
            except Exception as e:
                print(f"LiDAR scan error (attempt {attempt+1}/{retries}): {e}")
                try:
                    await asyncio.to_thread(self.lidar.stop)
                    await asyncio.to_thread(self.lidar.clear_input)
                except Exception:
                    pass
                await asyncio.sleep(0.5)
        return self.scan_data

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
        return self.lidar.get_info() if self.lidar else None

    def stop(self):
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.disconnect()
        except Exception as e:
            print(f"LiDAR stop error: {e}")