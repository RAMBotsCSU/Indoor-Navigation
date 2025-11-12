import os
from math import cos, sin, pi, floor
import sys
from PyQt6.QtGui import QPixmap, QPainter, QColor, QPen
from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import QApplication, QMainWindow
from adafruit_rplidar import RPLidar
import threading

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


if __name__ == "__main__":
    lidar = LiDAR('/dev/ttyUSB0')
    ui = UI()
    ui.start()

    # cleanup (if scanning was started, ensure lidar.stop/disconnect are called appropriately)
    try:
        lidar.stop()
        lidar.disconnect()
    except Exception:
        pass