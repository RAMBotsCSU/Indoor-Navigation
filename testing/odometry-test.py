import pygame
import odrive
from math import pi, cos, sin
import asyncio

odrv_enable = True
analog_keys = {0: 0, 1: 0, 2: 0, 3: 0}

# Wheel Radius is 15.5 cm
WHEEL_RADIUS = 15.5
WHEEL_CIRCUMFERENCE = 2 * pi * WHEEL_RADIUS
WHEEL_BASE = 100.0  # distance between wheels in cm

class ODrive:
    def __init__(self):
        if odrv_enable:
            self.odrv = odrive.find_any()
        self.controller_init()
    
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

    async def control(self):
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                analog_keys[event.axis] = event.value
                self.move_axis(analog_keys[0], 0)
                self.move_axis(analog_keys[1], 1)

    def read_motion(self, input):
        if abs(input) > 0.2:
            return input
        else: 
            return 0.0
        
    async def forward_move(self, pos):
        self.odrv.axis0.controller.input_vel = -1.0
        self.odrv.axis1.controller.input_vel = -1.0
        await asyncio.sleep(1)
        x = pos[0] + WHEEL_CIRCUMFERENCE
        y = pos[1] + WHEEL_CIRCUMFERENCE
        position = [x, y]
        return position
    
    # calculated turn movement
    async def turn_move(self, angle, theta):
        self.controller0.input_vel = self.sign(angle) * -1.0
        self.controller1.input_vel = self.sign(angle) * 1.0
        await asyncio.sleep(1)
        theta = self.sign(angle)*self.arc_length(WHEEL_RADIUS) + theta
        return theta
     
    def init_odometry(self):
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
    
    def sign(self, theta):
        if theta > 0:
            return 1
        else:
            return -1

    def arc_length(self,arc):
        return arc * 180 / (pi * WHEEL_BASE/2)
    

if __name__ == "__main__":
    home_position = [0.0,0.0]
    home_rotation = 0.0
    odrv = ODrive()
    try:
        odrv.odometry()
        home_position = odrv.forward_move(home_position)
        home_rotation = odrv.turn_move(home_rotation, 90)
        asyncio.run(odrv.control())
        
    except KeyboardInterrupt:
        print("Exiting")