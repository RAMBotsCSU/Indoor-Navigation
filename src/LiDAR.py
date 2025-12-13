import time
import threading
import asyncio
from adafruit_rplidar import RPLidar

class LiDAR:
    def __init__(self, port="/dev/ttyUSB0", max_distance=6000):
        self.port = port
        self.max_distance = max_distance
        self.lidar = RPLidar(None, port, timeout=3)
        self.queue = asyncio.Queue(maxsize=2000)
        self._running = False
        self._thread = None

    def start(self, loop):
        self._running = True
        self._loop = loop
        self.lidar.start_motor()
        self._thread = threading.Thread(
            target=self._scan_loop,
            daemon=True
        )
        self._thread.start()

    def _scan_loop(self):
        try:
            for new_scan, quality, angle, distance in self.lidar.iter_measurments():
                if not self._running:
                    break

                if distance <= 0 or distance > self.max_distance:
                    continue

                timestamp = time.monotonic()

                self._loop.call_soon_threadsafe(
                    self.queue.put_nowait,
                    (timestamp, angle, distance)
                )
        except Exception as e:
            print("LiDAR error:", e)

    async def stop(self):
        self._running = False
        await asyncio.sleep(0)
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()
