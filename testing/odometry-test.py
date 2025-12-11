import pygame
import odrive
from odrive.enums import *
from typing import Tuple
import math
import asyncio
from concurrent.futures import ThreadPoolExecutor

odrv_enable = True
analog_keys = {0: 0, 1: 0, 2: 0, 3: 0}

WHEEL_RADIUS = 15.5
WHEEL_CIRCUMFERENCE = 2 * math.pi * WHEEL_RADIUS
WHEEL_BASE = 59.0 
GEAR_RATIO = 1.0

# Arrival tuning
POS_TOL = 0.005
TIMEOUT = 8.0

class ODrive:
    def __init__(self):
        # don't initialize ODrive in __init__; do it async in async_init
        self.odrv = None
        self.controller_init()
        self.controller0 = None
        self.controller1 = None
    
    async def async_init(self):
        # find ODrive asynchronously
        if odrv_enable:
            loop = asyncio.get_event_loop()
            with ThreadPoolExecutor() as pool:
                self.odrv = await loop.run_in_executor(pool, odrive.find_any)
        return self

    def controller_init(self):
        pygame.init()
        pygame.joystick.init()
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        print("Found controller:", joystick.get_name())
        print("GUID: ", joystick.get_guid())
    
    def move_axis(self, input, axis):
        if odrv_enable:
            if axis == 0:
                self.odrv.axis0.controller.input_vel = -1.0*self.read_motion(input)
            if axis == 1:
                self.odrv.axis1.controller.input_vel = -1.0*self.read_motion(input)

    async def control_loop(self):
        # continuous async control loop reading pygame events
        clock = pygame.time.Clock()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.JOYAXISMOTION:
                    analog_keys[event.axis] = event.value
                    self.move_axis(analog_keys[0], 0)
                    self.move_axis(analog_keys[1], 1)
            # yield control back to asyncio event loop
            await asyncio.sleep(0.01)
            clock.tick(60)

    def read_motion(self, input):
        if abs(input) > 0.2:
            return input
        else: 
            return 0.0
        
    async def forward_move(self, pos):
        if odrv_enable and self.odrv:
            self.odrv.axis0.controller.input_vel = -1.0
            self.odrv.axis1.controller.input_vel = -1.0
        await asyncio.sleep(1)
        self.end_odometry()
        x = pos[0] + WHEEL_CIRCUMFERENCE
        y = pos[1] + WHEEL_CIRCUMFERENCE
        position = [x, y]
        return position
    
    async def turn_move(self, theta, angle):
        if odrv_enable and self.odrv:
            self.controller0.input_vel = self.sign(angle) * -1.0
            self.controller1.input_vel = self.sign(angle) * 1.0
        await asyncio.sleep(1)
        self.end_odometry()
        theta = self.sign(angle) * self.arc_length(WHEEL_RADIUS) + theta
        return theta
     
    def init_odometry(self):
        if not odrv_enable or not self.odrv:
            return
        x, y, theta = 0.0, 0.0, 0.0

        # init controllers
        self.controller0 = self.odrv.axis0.controller
        self.controller1 = self.odrv.axis1.controller

        # init control mode
        self.controller0.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL
        self.controller1.config.control_mode = odrive.enums.CONTROL_MODE_VELOCITY_CONTROL

        # set velocity gains
        self.controller0.config.vel_gain = 0.5
        self.controller1.config.vel_gain = 0.5

        # set input type
        self.controller0.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP
        self.controller1.config.input_mode = odrive.enums.INPUT_MODE_VEL_RAMP
            
    def odometry(self):
        self.init_odometry()
    
    def end_odometry(self):
        if not odrv_enable or not self.odrv:
            return
        self.controller0.input_vel = 0.0
        self.controller1.input_vel = 0.0

    def sign(self, theta):
        if theta > 0:
            return 1
        else:
            return -1

    def arc_length(self,arc):
        return arc * 180 / (math.pi * WHEEL_BASE/2)

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
        self.a0.controller.config.control_mode = odrive.enums.CONTROL_MODE_POSITION_CONTROL
        self.a1.controller.config.control_mode = odrive.enums.CONTROL_MODE_POSITION_CONTROL

        # input pos = current pos
        self.a0.controller.config.input_mode = odrive.enums.INPUT_MODE_PASSTHROUGH
        self.a1.controller.config.input_mode = odrive.enums.INPUT_MODE_PASSTHROUGH

        # sync the two
        self.a0.controller.input_pos = self.a0.encoder.pos_estimate
        self.a1.controller.input_pos = self.a1.encoder.pos_estimate

        # closed loop control
        self.a0.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL
        self.a1.requested_state = odrive.enums.AXIS_STATE_CLOSED_LOOP_CONTROL

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


async def main():
    bot = Odometry()
    await bot.connect()
    print("Connected")

    await bot.enable()
    print("Enabled")

    await bot.forward_cm(50)
    print("After forward 50cm:", bot.pose())

    await bot.turn_deg(90)
    print("After 90° turn:", bot.pose())

    bot.stop()

asyncio.run(main())