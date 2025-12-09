from __future__ import annotations

import asyncio

from typing import TYPE_CHECKING

import aiofiles

from aiopath import AsyncPath
from pathlib import Path

from evdev import InputDevice, InputEvent, ecodes, list_devices

if TYPE_CHECKING:
    from .sparky import Sparky

class ControllerLED:
    def __init__(self, base_path: AsyncPath) -> None:

        base_path = Path(base_path) # eugh

        if base_path.exists():
            for path in base_path.glob("*red*"):
                self.red = path.joinpath('brightness')

            for path in base_path.glob("*green*"):
                self.green = path.joinpath('brightness')

            for path in base_path.glob("*blue*"):
                self.blue = path.joinpath('brightness')

        else:
            print("Could not find LEDs")

    async def _write_color(self, path, value):
        async with aiofiles.open(path, mode='w') as f:
            await f.write(str(value))

    async def set_color(self, color: tuple):
        await self._write_color(self.red, color[0])
        await self._write_color(self.green, color[1])
        await self._write_color(self.blue, color[2])


class Controller:
    # Button states
    cross = 0
    circle = 0
    triangle = 0
    square = 0
    dpad_up = 0
    dpad_down = 0
    dpad_left = 0
    dpad_right = 0
    l1 = 0
    r1 = 0
    l2 = 0  # Digital press
    r2 = 0  # Digital press
    share = 0
    options = 0
    l3 = 0
    r3 = 0

    # Analog stick and trigger positions
    left_stick_x = 128
    left_stick_y = 128
    right_stick_x = 128
    right_stick_y = 128
    l2_analog = 0  # Analog trigger L2
    r2_analog = 0  # Analog trigger R2

    battery = 0


    def __init__(self, robot: Sparky) -> None:
        self.robot = robot

        self.dev_base_path = AsyncPath("/sys/class/input/js0/")

        self.dev = self._find_controller_dev()

        for event_type, events in self.dev.capabilities(absinfo=True).items():
            for event in events:
                if isinstance(event, tuple):
                    event_code, absinfo = event

                    self.update_event(InputEvent(sec=0, usec=0, type=ecodes.EV_ABS, code=event_code, value=absinfo.value))

        self.led = ControllerLED(self.dev_base_path.joinpath("device/device/leds/"))

        self.is_ready = asyncio.Event()

    def _find_controller_dev(self) -> InputDevice:
        for device in [InputDevice(path) for path in list_devices()]:
            # Print the name to see the device names
            print(f"Checking device: {device.name} - {device.info} - {device.path}")

            if "Motion" in device.name or "Touchpad" in device.name:
                continue  # for some reason it links to sub devices like the touchpad, so stop that

            # Check if this device is a PS4 controller by name
            if "Sony" in device.name or "Wireless Controller" in device.name:
                print(f"PS4 controller found: {device.name} at {device.path}")
                return device

        raise Exception("Controller not found.")

    def update_event(self, event: InputEvent):
        # Update button states for EV_KEY events (buttons)
        if event.type == ecodes.EV_KEY:
            if event.code == ecodes.BTN_SOUTH:  # Cross
                self.cross = event.value
            elif event.code == ecodes.BTN_EAST:  # Circle
                self.circle = event.value
            elif event.code == ecodes.BTN_NORTH:  # Triangle
                self.triangle = event.value
            elif event.code == ecodes.BTN_WEST:  # Square
                self.square = event.value
            elif event.code == ecodes.BTN_DPAD_UP:
                self.dpad_up = event.value
            elif event.code == ecodes.BTN_DPAD_DOWN:
                self.dpad_down = event.value
            elif event.code == ecodes.BTN_DPAD_LEFT:
                self.dpad_left = event.value
            elif event.code == ecodes.BTN_DPAD_RIGHT:
                self.dpad_right = event.value
            elif event.code == ecodes.BTN_TL:  # L1
                self.l1 = event.value
            elif event.code == ecodes.BTN_TR:  # R1
                self.r1 = event.value
            elif event.code == ecodes.BTN_TL2:  # L2 digital
                self.l2 = event.value
            elif event.code == ecodes.BTN_TR2:  # R2 digital
                self.r2 = event.value
            elif event.code == ecodes.BTN_SELECT:  # Share
                self.share = event.value
            elif event.code == ecodes.BTN_START:  # Options
                self.options = event.value
            elif event.code == ecodes.BTN_THUMBL:  # L3 (left stick press)
                self.l3 = event.value
            elif event.code == ecodes.BTN_THUMBR:  # R3 (right stick press)
                self.r3 = event.value
            else:
                print(f" - UNHANDLED EV_KEY: {event.code} {ecodes.KEY[event.code]}")

        # Update analog stick and trigger positions for EV_ABS events
        elif event.type == ecodes.EV_ABS:
            if event.code == ecodes.ABS_X:  # Left stick X axis
                self.left_stick_x = event.value
            elif event.code == ecodes.ABS_Y:  # Left stick Y axis
                self.left_stick_y = event.value
            elif event.code == ecodes.ABS_RX:  # Right stick X axis
                self.right_stick_x = event.value
            elif event.code == ecodes.ABS_RY:  # Right stick Y axis
                self.right_stick_y = event.value
            elif event.code == ecodes.ABS_Z:  # L2 trigger analog
                self.l2_analog = event.value
            elif event.code == ecodes.ABS_RZ:  # R2 trigger analog
                self.r2_analog = event.value
            elif event.code == ecodes.ABS_HAT0X:  # D-pad X axis
                if event.value == -1:
                    self.dpad_left = 1
                    self.dpad_right = 0
                elif event.value == 1:
                    self.dpad_right = 1
                    self.dpad_left = 0
                else:
                    self.dpad_left = 0
                    self.dpad_right = 0
            elif event.code == ecodes.ABS_HAT0Y:  # D-pad Y axis
                if event.value == -1:
                    self.dpad_up = 1
                    self.dpad_down = 0
                elif event.value == 1:
                    self.dpad_down = 1
                    self.dpad_up = 0
                else:
                    self.dpad_up = 0
                    self.dpad_down = 0
            else:
                print(f" - UNHANDLED EV_ABS: {event.code} {ecodes.ABS[event.code]}")


    async def events(self):
        try:
            async for event in self.dev.async_read_loop():
                self.update_event(event)

        except OSError:
            print("Controller Disconnected")
            await self.robot.set_enabled(False)

        except asyncio.CancelledError:
            print("Cancelled")

        except Exception as e:
            print(e)

    async def polling(self):
        try:
            while True:
                # battery percentage
                async for path in self.dev_base_path.joinpath("device/device/power_supply/").glob("*/capacity"):
                    async with aiofiles.open(path) as fp:
                        self.battery = int(await fp.read())

                await asyncio.sleep(60)

        except asyncio.CancelledError:
            print("EEEEEEEEE")
            return

        except Exception as e:
            print(e)


    def stop(self): ...
        # self._stop.set()

    def __str__(self):
        return (f"Cross: {self.cross}, Circle: {self.circle}, Triangle: {self.triangle}, Square: {self.square}\n"
                f"D-Pad Up: {self.dpad_up}, Down: {self.dpad_down}, Left: {self.dpad_left}, Right: {self.dpad_right}\n"
                f"L1: {self.l1}, R1: {self.r1}, L2: {self.l2}, R2: {self.r2}\n"
                f"Left Stick: ({self.left_stick_x}, {self.left_stick_y}), "
                f"Right Stick: ({self.right_stick_x}, {self.right_stick_y})\n"
                f"L2 Analog: {self.l2_analog}, R2 Analog: {self.r2_analog}\n"
                f"Share: {self.share}, Options: {self.options}, L3: {self.l3}, R3: {self.r3}\n"
                f"Battery: {self.battery}%")
