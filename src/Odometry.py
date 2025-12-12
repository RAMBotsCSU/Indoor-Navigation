import odrive
import asyncio
import math
from typing import Tuple
# FIX: import enums explicitly; odrive may not expose `enums` attribute
from odrive.enums import (
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
)

WHEEL_RADIUS = 15.5
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
WHEEL_BASE = 59.0 
GEAR_RATIO = 1.0

# Arrival tuning
POS_TOL = 0.005
TIMEOUT = 8.0

class Odometry:
    def __init__(self):
        self.odrv = None
        self.a0 = None
        self.a1 = None

        self.x = 0.0
        self.y = 0.0
        self.th = 0.0  # radians

        self.last0 = 0.0
        self.last1 = 0.0

    async def connect(self):
        """Find ODrive asynchronously."""
        loop = asyncio.get_event_loop()
        self.odrv = await loop.run_in_executor(None, odrive.find_any)
        self.a0 = self.odrv.axis0
        self.a1 = self.odrv.axis1
        return self

    async def enable(self):
        # position control
        self.a0.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        self.a1.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL

        # input pos = current pos
        self.a0.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
        self.a1.controller.config.input_mode = INPUT_MODE_PASSTHROUGH

        # sync the two
        self.a0.controller.input_pos = self.a0.encoder.pos_estimate
        self.a1.controller.input_pos = self.a1.encoder.pos_estimate

        # closed loop control
        self.a0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.a1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        # set start point with encoders
        await asyncio.sleep(0.1)
        self.last0 = float(self.a0.encoder.pos_estimate)
        self.last1 = float(self.a1.encoder.pos_estimate)

    # Simple movement function
    async def _move_to(self, target0, target1, timeout=TIMEOUT):
        self.a0.controller.input_pos = target0
        self.a1.controller.input_pos = target1

        t0 = asyncio.get_event_loop().time()

        while True:
            p0 = float(self.a0.encoder.pos_estimate)
            p1 = float(self.a1.encoder.pos_estimate)

            if abs(p0 - target0) <= POS_TOL and abs(p1 - target1) <= POS_TOL:
                self._update_odometry(p0, p1)
                return True

            if asyncio.get_event_loop().time() - t0 > timeout:
                self._update_odometry(p0, p1)
                return False

            await asyncio.sleep(0.01)

    # Move forward a specific distance in cm
    async def forward_cm(self, distance_cm: float):
        # wheel revolutions
        wheel_revs = distance_cm / WHEEL_CIRCUMFERENCE
        motor_turns = wheel_revs * GEAR_RATIO

        start0 = float(self.a0.encoder.pos_estimate)
        start1 = float(self.a1.encoder.pos_estimate)

        return await self._move_to(start0 - motor_turns,
                                   start1 + motor_turns)

    async def turn_deg(self, angle_deg: float):
        theta_rad = math.radians(angle_deg)

        # arc per wheel (cm). Left = -arc, right = +arc for CCW.
        arc_cm = (theta_rad * WHEEL_BASE) / 2.0
        wheel_revs = arc_cm / WHEEL_CIRCUMFERENCE
        motor_turns = wheel_revs * GEAR_RATIO

        start0 = float(self.a0.encoder.pos_estimate)
        start1 = float(self.a1.encoder.pos_estimate)

        return await self._move_to(start0 + motor_turns,
                                   start1 + motor_turns)

    # update odometry
    def _update_odometry(self, p0: float, p1: float):
        d0 = p0 - self.last0
        d1 = p1 - self.last1

        self.last0 = p0
        self.last1 = p1

        # convert motor turns to wheel linear displacement
        dL = (d0 / GEAR_RATIO) * WHEEL_CIRCUMFERENCE
        dR = (d1 / GEAR_RATIO) * WHEEL_CIRCUMFERENCE

        d_center = (dL + dR) / 2.0
        d_theta = (dR - dL) / WHEEL_BASE

        # integrate
        self.x += d_center * math.cos(self.th + d_theta / 2.0)
        self.y += d_center * math.sin(self.th + d_theta / 2.0)
        self.th += d_theta

    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.th

    def stop(self):
        # hold current position
        p0 = float(self.a0.encoder.pos_estimate)
        p1 = float(self.a1.encoder.pos_estimate)
        self.a0.controller.input_pos = p0
        self.a1.controller.input_pos = p1