import asyncio
import time
import math
from collections import deque
from typing import Tuple
import odrive

# Robot parameters (cm)
WHEEL_RADIUS = 15.5
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
WHEEL_BASE = 59.0
GEAR_RATIO = 1.0

class OdometryEstimator:
    def __init__(self, history_size: int = 5000):
        # ODrive
        self.odrv = None
        self.a0 = None
        self.a1 = None

        # Pose state (cm, cm, rad)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Encoder state
        self.last0 = 0.0
        self.last1 = 0.0
        self.last_time = None

        # Pose history for interpolation
        self.history = deque(maxlen=history_size)

        self._running = False

    async def connect(self):
        loop = asyncio.get_event_loop()
        self.odrv = await loop.run_in_executor(None, odrive.find_any)
        self.a0 = self.odrv.axis0
        self.a1 = self.odrv.axis1

        # Initialize encoder baselines
        self.last0 = float(self.a0.encoder.pos_estimate)
        self.last1 = float(self.a1.encoder.pos_estimate)
        self.last_time = time.monotonic()

        # Store initial pose
        self.history.append((self.last_time, self.x, self.y, self.th))
        return self

    async def start(self, rate_hz: int = 200):
        """
        Start continuous odometry updates.
        """
        self._running = True
        period = 1.0 / rate_hz

        while self._running:
            self._update_from_encoders()
            await asyncio.sleep(period)

    def stop(self):
        self._running = False

    def _update_from_encoders(self):
        now = time.monotonic()

        p0 = float(self.a0.encoder.pos_estimate)
        p1 = float(self.a1.encoder.pos_estimate)

        # encoder deltas
        d0 = p0 - self.last0
        d1 = p1 - self.last1

        self.last0 = p0
        self.last1 = p1

        # convert to cm
        # axis0 spins opposite of axis1 for forward motion
        dL = (-d0 / GEAR_RATIO) * WHEEL_CIRCUMFERENCE  # flip sign for forward
        dR = (d1 / GEAR_RATIO) * WHEEL_CIRCUMFERENCE

        # center displacement and rotation
        d_center = (dL + dR) / 2.0
        d_theta = (dR - dL) / WHEEL_BASE*2

        # integrate pose
        self.x += d_center * math.cos(self.th + d_theta / 2.0)
        self.y += d_center * math.sin(self.th + d_theta / 2.0)
        self.th += d_theta

        self.last_time = now
        self.history.append((now, self.x, self.y, self.th))



    def pose(self) -> Tuple[float, float, float]:
        """Current pose."""
        return self.x, self.y, self.th

    def interpolate(self, timestamp: float) -> Tuple[float, float, float]:
        """
        Interpolate pose at an arbitrary timestamp (for LiDAR fusion).
        """
        if len(self.history) < 2:
            return self.x, self.y, self.th

        for i in range(len(self.history) - 1):
            t0, x0, y0, th0 = self.history[i]
            t1, x1, y1, th1 = self.history[i + 1]

            if t0 <= timestamp <= t1:
                alpha = (timestamp - t0) / (t1 - t0)
                x = x0 + alpha * (x1 - x0)
                y = y0 + alpha * (y1 - y0)
                th = th0 + alpha * (th1 - th0)
                return x, y, th

        # If timestamp is newer than history
        return self.history[-1][1:]
