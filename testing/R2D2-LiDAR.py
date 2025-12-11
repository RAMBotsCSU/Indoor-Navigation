from math import cos, sin, pi, floor
import sys, time, asyncio, odrive, pygame, csv
from PyQt6.QtGui import QPixmap, QPainter, QColor, QPen
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMainWindow
from adafruit_rplidar import RPLidar

odrv_enable = True
analog_keys = {0: 0, 1: 0, 2: 0, 3: 0}

# Wheel Radius is 15.5 cm
WHEEL_RADIUS = 15.5
WHEEL_CIRCUMFERENCE = 2 * pi * WHEEL_RADIUS
WHEEL_BASE = 30.0  # distance between wheels in cm

class UI:
    def __init__(self):
        self.app = QApplication(sys.argv)
        self.window = QMainWindow()
        self.window.setWindowTitle("LiDAR Viewer")
        self.window.resize(800, 800)

    def start(self):
        self.window.show()
        self.app.exec()

class LiDAR:    
    def __init__(self, port='/dev/ttyUSB0', max_distance=6000):
        self.port = port
        self.max_distance = max_distance
        self.lidar = RPLidar(None, port, timeout=3)
        self.data = []

    def process_data(self, data):
        for angle in range(360):
            distance = data[angle]
            if distance > 0:
                capped = min(self.max_distance, distance)
                radians = angle * pi / 180.0
                x = capped * cos(radians)
                y = capped * sin(radians)
    
    def plot_data(self, data):
        pixmap = QPixmap(800, 800)
        pixmap.fill(Qt.GlobalColor.white)
        painter = QPainter(pixmap)
        pen = QPen(QColor("black"))
        pen.setWidth(2)
        painter.setPen(pen)
        for angle in range(360):
            distance = data[angle]
            if distance > 0:
                radians = angle * pi / 180.0
                x = 400 + distance * cos(radians) / 10
                y = 400 + distance * sin(radians) / 10
                painter.drawLine(400, 400, x, y)
        painter.end()
        return pixmap

    def info(self):
        return self.lidar.get_info()

    def save_data(self, data):
        filename = self.get_filename()
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for row in data:
                writer.writerow([row])
        print(f"LiDAR data saved to {filename}")
    
    def get_filename(self):
        date = time.strftime("%Y%m%d-%H%M%S")
        return "outputs/lidar_data_{}.csv".format(date)

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

async def main():
    task = asyncio.create_task(odrive.control())
    await task

if __name__ == "__main__":
    try:
        lidar = LiDAR('/dev/ttyUSB0')
        ui = UI()
        ui.start()

        odrive = ODrive()
        asyncio.run(main())

    except KeyboardInterrupt:
        lidar.stop()
        lidar.disconnect()
        print("Exiting")
        lidar.save_data()