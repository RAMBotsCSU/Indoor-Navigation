# SPDX-FileCopyrightText: 2019 Dave Astels for Adafruit Industries
#
# SPDX-License-Identifier: MIT

"""
Consume LIDAR measurement file and create an image for display.

Adafruit invests time and resources providing this open source code.
Please support Adafruit and open source hardware by purchasing
products from Adafruit!

Written by Dave Astels for Adafruit Industries
Copyright (c) 2019 Adafruit Industries
Licensed under the MIT license.

All text above must be included in any redistribution.

"""

import os
import sys
from math import cos, sin, pi, floor
from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtCore import pyqtSignal, QThread
from PyQt6.QtGui import QColor, QPainter
from adafruit_rplidar import RPLidar

PORT_NAME = '/dev/ttyUSB0'

class LidarWorker(QThread):
    scan_ready = pyqtSignal(list)
    def __init__(self, port=PORT_NAME, parent=None):
        super().__init__(parent)
        self.port = port
        self._running = True
        self.lidar = None

    def run(self):
        try:
            # Initialize lidar inside thread
            self.lidar = RPLidar(None, self.port, timeout=3)  # keep original constructor usage
            for scan in self.lidar.iter_scans():
                if not self._running:
                    break
                scan_data = [0] * 360
                for (_, angle, distance) in scan:
                    scan_data[min(359, floor(angle))] = distance
                self.scan_ready.emit(scan_data)
        except Exception as e:
            # print minimal error info
            print("Lidar worker error:", e)
        finally:
            try:
                if self.lidar:
                    self.lidar.stop()
                    self.lidar.disconnect()
            except Exception:
                pass

    def stop(self):
        self._running = False
        # stopping the iterator from another thread by closing device is handled in finally
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.disconnect()
        except Exception:
            pass
        self.wait(2000)


class ScanWidget(QtWidgets.QWidget):
    def __init__(self, parent=None, size=600):
        super().__init__(parent)
        self.setFixedSize(size, size)
        self.size = size
        self.scan_data = [0] * 360
        self.max_distance = 0

    @QtCore.pyqtSlot(list)
    def update_scan(self, data):
        # store reference copy
        self.scan_data = data[:]  # shallow copy
        # update max_distance similar to original code: ensure non-decreasing to stabilize scaling
        for d in data:
            if d > 0:
                self.max_distance = max(min(5000, d), self.max_distance)
        # trigger repaint
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(0, 0, self.size, self.size, QColor(0, 0, 0))
        if self.max_distance <= 0:
            return
        cx = cy = self.size // 2
        scale = 119.0 / self.max_distance
        pen = QtGui.QPen(QColor(255, 255, 255))
        pen.setWidth(1)
        painter.setPen(pen)
        for angle in range(360):
            distance = self.scan_data[angle]
            if distance > 0:
                radians = angle * pi / 180.0
                x = distance * cos(radians)
                y = distance * sin(radians)
                px = cx + int(x * scale)
                py = cy + int(y * scale)
                # draw a single point (small rectangle for visibility)
                painter.drawPoint(px, py)
        painter.end()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("RPLidar Viewer (PyQt6)")
        self.widget = ScanWidget(self, size=600)
        self.setCentralWidget(self.widget)

        self.worker = LidarWorker(PORT_NAME)
        self.worker.scan_ready.connect(self.widget.update_scan)
        self.worker.start()

    def closeEvent(self, event):
        try:
            self.worker.stop()
        except Exception:
            pass
        event.accept()


def main():
    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()