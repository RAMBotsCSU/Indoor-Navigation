import time
import threading
import asyncio
from adafruit_rplidar import RPLidar

class LiDAR:
    def __init__(self, port="/dev/ttyUSB0", max_distance=6000, test_output=True, queue_maxsize=500):
        """
        port: serial port of the LiDAR
        max_distance: maximum distance to accept in mm
        test_output: if True, prints some sample points to confirm
        """
        self.port = port
        self.max_distance = max_distance
        self.lidar = None
        self._running = False
        self._thread = None
        self._loop = None
        self.queue = asyncio.Queue(maxsize=queue_maxsize)
        self.test_output = test_output

    async def connect(self):
        """Initialize LiDAR and prepare for scanning"""
        try:
            print("Connecting to LiDAR...")
            self.lidar = RPLidar(None, self.port, timeout=3)
        except Exception as e:
            raise RuntimeError(f"Failed to initialize LiDAR: {e}")

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
        """Start LiDAR motor and scanning thread"""
        if not self.lidar:
            raise RuntimeError("LiDAR not connected. Call connect() first.")
        self._loop = loop
        self._running = True

        # Start motor
        try:
            self.lidar.start_motor()
            print("LiDAR motor started")
        except Exception as e:
            raise RuntimeError(f"Failed to start LiDAR motor: {e}")

        time.sleep(1.0)  # let motor spin up

        # Start scanning thread
        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _enqueue(self, item):
        """Runs on loop thread: drop oldest if full, then enqueue."""
        if self.queue.full():
            try:
                self.queue.get_nowait()
            except asyncio.QueueEmpty:
                pass
        try:
            self.queue.put_nowait(item)
        except asyncio.QueueFull:
            pass

    def _scan_loop(self):
        """Read measurements from LiDAR and push to asyncio queue"""
        print("LiDAR scanning thread started")
        sample_count = 0
        try:
            for new_scan, quality, angle, distance in self.lidar.iter_measurments():
                if not self._running:
                    break

                if distance <= 0 or distance > self.max_distance:
                    continue

                ts = time.monotonic()
                try:
                    self._loop.call_soon_threadsafe(self._enqueue, (ts, angle, distance))
                except Exception:
                    pass

                # Print a few points to confirm
                if self.test_output and sample_count < 5:
                    print(f"LiDAR sample: angle={angle:.1f}, distance={distance:.1f} mm")
                    sample_count += 1

        except Exception as e:
            print("LiDAR thread exited:", e)
        finally:
            print("LiDAR scanning thread ended")

    async def stop(self):
        """Stop scanning and motor"""
        self._running = False
        await asyncio.sleep(0.1)  # let thread exit
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.stop_motor()
                self.lidar.disconnect()
                print("LiDAR stopped and disconnected")
        except Exception as e:
            print("Error stopping LiDAR:", e)
