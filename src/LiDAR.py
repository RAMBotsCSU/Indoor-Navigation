import time
import threading
import asyncio
from adafruit_rplidar import RPLidar

class LiDAR:
    def __init__(self, port="/dev/ttyUSB0", max_distance=6000):
        self.port = port
        self.max_distance = max_distance
        self.lidar = None
        self.queue = asyncio.Queue(maxsize=2000)
        self._running = False
        self._thread = None
        self._loop = None

    async def connect(self):
        """Initialize LiDAR connection and prepare device."""
        # Create RPLidar instance
        self.lidar = RPLidar(None, self.port, timeout=3)
        
        # Stop motor & clear input
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
        except Exception:
            pass

        await asyncio.sleep(0.5)

        try:
            self.lidar.clear_input()
        except Exception:
            pass

        await asyncio.sleep(0.2)
        print("LiDAR connected and ready")

    def start(self, loop):
        """Start LiDAR motor and scanning thread."""
        self._loop = loop
        self._running = True

        # Start motor
        self.lidar.start_motor()
        time.sleep(1.0)  # let motor spin up

        # Start scanning thread
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        """Read measurements from LiDAR and push to asyncio queue."""
        try:
            for new_scan, quality, angle, distance in self.lidar.iter_measurments():
                if not self._running:
                    break

                if distance <= 0 or distance > self.max_distance:
                    continue

                timestamp = time.monotonic()
                # Push to asyncio queue thread-safely
                self._loop.call_soon_threadsafe(
                    self.queue.put_nowait, (timestamp, angle, distance)
                )
        except Exception as e:
            print("LiDAR error:", e)

    async def stop(self):
        """Stop scanning and motor."""
        self._running = False
        await asyncio.sleep(0)  # allow thread to exit
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except Exception as e:
            print("LiDAR stop error:", e)
