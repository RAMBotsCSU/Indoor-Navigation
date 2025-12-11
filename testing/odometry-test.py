import pygame
import odrive
from odrive.enums import CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_VELOCITY_CONTROL
from math import pi, cos, sin
import asyncio
from concurrent.futures import ThreadPoolExecutor

odrv_enable = True
analog_keys = {0: 0, 1: 0, 2: 0, 3: 0}

# Wheel Radius is 15.5 cm
WHEEL_RADIUS = 15.5
WHEEL_CIRCUMFERENCE = 2 * pi * WHEEL_RADIUS
WHEEL_BASE = 59.0  # distance between wheels in cm

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
        x = pos[0] + WHEEL_CIRCUMFERENCE
        y = pos[1] + WHEEL_CIRCUMFERENCE
        position = [x, y]
        return position
    
    async def turn_move(self, theta, angle):
        if odrv_enable and self.odrv:
            self.controller0.input_vel = self.sign(angle) * -1.0
            self.controller1.input_vel = self.sign(angle) * 1.0
        await asyncio.sleep(1)
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
        self.controller0.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.controller1.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        # set velocity gains
        self.controller0.config.vel_gain = 0.5
        self.controller1.config.vel_gain = 0.5

        # set input type
        self.controller0.config.input_mode = INPUT_MODE_VELOCITY_CONTROL
        self.controller1.config.input_mode = INPUT_MODE_VELOCITY_CONTROL
            
    def odometry(self):
        self.init_odometry()
    
    def sign(self, theta):
        if theta > 0:
            return 1
        else:
            return -1

    def arc_length(self,arc):
        return arc * 180 / (pi * WHEEL_BASE/2)
    

async def async_main():
    home_position = [0.0, 0.0]
    home_rotation = 0.0
    odrv = ODrive()
    # initialize ODrive hardware asynchronously
    await odrv.async_init()
    
    try:
        odrv.init_odometry()
        # run forward and turn moves sequentially
        home_position = await odrv.forward_move(home_position)
        print("New position:", home_position)
        home_rotation = await odrv.turn_move(home_rotation, 90)
        print("New rotation:", home_rotation)
        
        # start continuous control loop
        await odrv.control_loop()
        
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        pygame.quit()

if __name__ == "__main__":
    asyncio.run(async_main())