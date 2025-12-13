import time
import threading
import asyncio
from adafruit_rplidar import RPLidar

class LiDAR:
    """
    Async-compatible RPLidar class with a thread reading measurements
    and pushing (timestamp, angle, distance) into an asyncio.Queue.
    """

    def __init__(self, port="/dev/ttyUSB0", max_distance=6000, queue_size=2000):
        self.port = port
        self.max_distance = max_distance
        self.queue = asyncio.Queue(maxsize=queue_size)
        self._running = False
        self._thread = None
        self._loop = None
        self.lidar = None

    def start(self, loop):
        """Start LiDAR streaming in a background thread."""
        self._loop = loop
        self._running = True

        # Initialize and hard-reset LiDAR
        self.lidar = RPLidar(None, self.port, timeout=3)

        try:
            self.lidar.stop()
            self.lidar.stop_motor()
        except Exception:
            pass

        time.sleep(0.5)

        try:
            self.lidar.clear_input()
        except Exception:
            pass

        time.sleep(0.2)
        self.lidar.start_motor()
        time.sleep(1.0)

        # Start thread for continuous measurement
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        try:
            for new_scan, quality, angle, distance in self.lidar.iter_measurments():
                if not self._running:
                    break

                if distance <= 0 or distance > self.max_distance:
                    continue

                timestamp = time.monotonic()

                # Push into asyncio queue thread-safely
                if not self.queue.full():
                    self._loop.call_soon_threadsafe(
                        self.queue.put_nowait,
                        (timestamp, angle, distance)
                    )
        except Exception as e:
            print("LiDAR error:", e)

    def stop(self):
        """Stop LiDAR streaming and safely disconnect."""
        self._running = False
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
        except Exception:
            pass
