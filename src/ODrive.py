import asyncio
import odrive

class ODrive:
    def __init__(self):
        # don't initialize ODrive in __init__; do it async in async_init
        self.odrv = None
        self.controller_init()
        self.controller0 = None
        self.controller1 = None
        self.odrv_enable = True
        self.odrv = odrive.find_any()

    def move_axis(self, input, axis):
        if self.odrv_enable:
            if axis == 0:
                self.odrv.axis0.controller.input_vel = -1.0*self.read_motion(input)
            if axis == 1:
                self.odrv.axis1.controller.input_vel = -1.0*self.read_motion(input)

    def stop(self):
        if self.odrv_enable:
            self.odrv.axis0.controller.input_vel = 0.0
            self.odrv.axis1.controller.input_vel = 0.0

    def read_motion(self, input):
        if abs(input) > 0.2:
            return input
        else: 
            return 0.0