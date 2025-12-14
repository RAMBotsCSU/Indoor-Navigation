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
        """Initialize LiDAR connection and reset device."""
        self.lidar = RPLidar(None, self.port, timeout=3)

        # Hard reset and clear input
        try:
            self.lidar.stop()
            self.lidar.stop_motor()
            time.sleep(0.5)
            self.lidar.clear_input()
            time.sleep(0.5)
        except Exception:
            pass

        print("LiDAR connected and reset")

    def start(self, loop):
        """Start motor and scanning thread."""
        if self.lidar is None:
            raise RuntimeError("LiDAR not connected")
        self._loop = loop
        self._running = True

        # Start motor
        self.lidar.start_motor()
        time.sleep(1.5)  # IMPORTANT: allow motor spinup

        # Start scan thread
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        """Push LiDAR measurements to asyncio queue."""
        try:
            iterator = self.lidar.iter_measurements()
            while self._running:
                try:
                    new_scan, quality, angle, distance = next(iterator)
                except StopIteration:
                    break
                except Exception:
                    continue

                if distance <= 0 or distance > self.max_distance:
                    continue

                ts = time.monotonic()
                try:
                    self._loop.call_soon_threadsafe(
                        self.queue.put_nowait, (ts, angle, distance)
                    )
                except asyncio.QueueFull:
                    pass

        except Exception as e:
            print("LiDAR thread exited:", e)

    async def stop(self):
        """Stop scanning and motor."""
        self._running = False
        await asyncio.sleep(0)
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except Exception as e:
            print("LiDAR stop error:", e)
