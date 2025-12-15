import asyncio
import math
import odrive
from odrive.enums import (
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_PASSTHROUGH,
    AXIS_STATE_CLOSED_LOOP_CONTROL,
)

# --- Robot constants ---
WHEEL_RADIUS_CM = 15.5        # wheel radius
WHEEL_BASE_CM = 59.0          # distance between wheels
GEAR_RATIO = 1.0              # motor-to-wheel turns
POS_TOL = 0.05               # position tolerance in turns
TIMEOUT_SEC = 8.0             # max wait per motion

WHEEL_CIRCUMFERENCE_CM = 2 * math.pi * WHEEL_RADIUS_CM


class RobotController:
    def __init__(self, verbose=False, update_rate_hz=200):
        self.odrv = None
        self.axis0 = None
        self.axis1 = None
        self.last_pos0 = 0.0
        self.last_pos1 = 0.0
        self.verbose = verbose
        self.update_delay = 1.0 / update_rate_hz  # Convert Hz to seconds

    async def connect(self):
        loop = asyncio.get_event_loop()
        self.odrv = await loop.run_in_executor(None, odrive.find_any)
        self.axis0 = self.odrv.axis0
        self.axis1 = self.odrv.axis1
        if self.verbose:
            print("ODrive connected")

    async def enable(self):
        for ax in [self.axis0, self.axis1]:
            ax.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
            ax.controller.config.input_mode = INPUT_MODE_PASSTHROUGH
            ax.controller.input_pos = ax.encoder.pos_estimate
            ax.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        await asyncio.sleep(0.1)
        self.last_pos0 = float(self.axis0.encoder.pos_estimate)
        self.last_pos1 = float(self.axis1.encoder.pos_estimate)
        if self.verbose:
            print(f"Motors enabled, positions: {self.last_pos0}, {self.last_pos1}")

    async def _move_to(self, target0, target1, timeout=TIMEOUT_SEC):
        self.axis0.controller.input_pos = target0
        self.axis1.controller.input_pos = target1

        t0 = asyncio.get_event_loop().time()
        while True:
            p0 = float(self.axis0.encoder.pos_estimate)
            p1 = float(self.axis1.encoder.pos_estimate)

            if abs(p0 - target0) <= POS_TOL and abs(p1 - target1) <= POS_TOL:
                self.last_pos0 = p0
                self.last_pos1 = p1
                if self.verbose:
                    print(f"Reached targets: {p0:.3f}, {p1:.3f}")
                return True

            if asyncio.get_event_loop().time() - t0 > timeout:
                self.last_pos0 = p0
                self.last_pos1 = p1
                if self.verbose:
                    print(f"Timeout reached: {p0:.3f}, {p1:.3f}")
                return False

            await asyncio.sleep(self.update_delay)  # Use configurable delay

    # Forward/backward movement
    async def forward_cm(self, distance_cm: float):
        wheel_turns = distance_cm / WHEEL_CIRCUMFERENCE_CM * GEAR_RATIO
        target0 = float(self.axis0.encoder.pos_estimate) - wheel_turns
        target1 = float(self.axis1.encoder.pos_estimate) + wheel_turns
        return await self._move_to(target0, target1)

    async def turn_deg(self, angle_deg: float):
        theta_rad = math.radians(angle_deg)
        arc_cm = theta_rad * WHEEL_BASE_CM  # Fixed: removed division by 2
        wheel_turns = arc_cm / WHEEL_CIRCUMFERENCE_CM * GEAR_RATIO
        target0 = float(self.axis0.encoder.pos_estimate) + wheel_turns
        target1 = float(self.axis1.encoder.pos_estimate) + wheel_turns
        return await self._move_to(target0, target1)

    async def move_arc(self, radius_cm: float, angle_deg: float, forward=True):
        theta_rad = math.radians(angle_deg)
        dL = theta_rad * (radius_cm - WHEEL_BASE_CM / 2.0)
        dR = theta_rad * (radius_cm + WHEEL_BASE_CM / 2.0)
        if not forward:
            dL *= -1
            dR *= -1
        target0 = float(self.axis0.encoder.pos_estimate) + dL / WHEEL_CIRCUMFERENCE_CM * GEAR_RATIO
        target1 = float(self.axis1.encoder.pos_estimate) + dR / WHEEL_CIRCUMFERENCE_CM * GEAR_RATIO
        return await self._move_to(target0, target1)

    async def stop(self):
        p0 = float(self.axis0.encoder.pos_estimate)
        p1 = float(self.axis1.encoder.pos_estimate)
        self.axis0.controller.input_pos = p0
        self.axis1.controller.input_pos = p1
        if self.verbose:
            print(f"Stopped motors at {p0:.3f}, {p1:.3f}")

    async def forward_cm_interpolated(self, distance_cm: float, speed_cm_s: float = 10.0):
        """Move forward with interpolated speed using small steps."""
        total_steps = max(1, int(abs(distance_cm) / speed_cm_s * 10))  # 10 updates per second
        step_cm = distance_cm / total_steps
        for _ in range(total_steps):
            await self.forward_cm(step_cm)
            await asyncio.sleep(0.1)  # 10 Hz update rate
