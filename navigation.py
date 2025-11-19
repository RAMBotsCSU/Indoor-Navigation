from PyQt6 import QtWidgets, QtCore, QtGui
from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QPixmap, QBrush, QColor, QImage, QPen, QPainterPath, QPainter
from PyQt6.QtCore import QTimer, QThread, pyqtSignal, Qt
try:
    from adafruit_rplidar import RPLidar
except Exception:
    RPLidar = None

import sys, random
import heapq, math, os
from math import cos, sin, pi, floor

red_threshold = 20
green_threshold = 20
blue_threshold = 20

PORT_NAME = '/dev/ttyUSB0' # usb device name
max_distance = 0  # max LiDAR distance in mm

class LiDAR(QThread):
    scan_ready = pyqtSignal(list)
    def __init__(self, port=PORT_NAME):
        super().__init__()
        # don't create hardware in constructor; do it in run()
        self.port = port
        self._running = True
        self.lidar = None
        self.scan_data = [0]*360

    def run(self):
        try:
            if RPLidar is not None:
                self.lidar = RPLidar(None, self.port, timeout=3)
                for scan in self.lidar.iter_scans():
                    if not self._running:
                        break
                    scan_data = [0] * 360
                    for (_, angle, distance) in scan:
                        scan_data[min(359, math.floor(angle))] = distance
                    self.scan_data = scan_data
                    # Emit the latest scan for UI
                    self.scan_ready.emit(self.scan_data)
            else:
                # fallback simulator loop if no hardware package available
                angle_offset = 0.0
                while self._running:
                    scan = [0] * 360
                    for a in range(360):
                        a_rel = (a + angle_offset) % 360
                        d = 0
                        if 45 < a_rel < 55:
                            d = 2000
                        if 225 < a_rel < 235:
                            d = 2500
                        if random.random() < 0.01:
                            d = random.randint(300, 4000)
                        scan[a] = d
                    self.scan_data = scan
                    self.scan_ready.emit(self.scan_data)
                    angle_offset += 2.0
                    self.msleep(50)
        except Exception as e:
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
        try:
            if self.lidar:
                self.lidar.stop()
                self.lidar.disconnect()
        except Exception:
            pass
        # wait for thread to finish gracefully
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


class IndoorNavigationViewer(QGraphicsView):
    def __init__(self, map_path, bbox, lidar):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # import map image
        self.map_pixmap = QPixmap(map_path)
        self.scene.addPixmap(self.map_pixmap)

        # bounding box
        # (min_x, max_x, min_y, max_y)
        self.bbox = bbox

        # User marker
        self.marker = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.marker.setBrush(QBrush(QColor("blue")))
        self.scene.addItem(self.marker)

        # Initial position
        self.x = (bbox[0] + bbox[1]) / 2
        self.y = (bbox[2] + bbox[3]) / 2
        self.update_marker()

        # simulated position updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position)
        self.timer.start(1000)

        # coarse grid cell size in pixels (tune for speed/precision)
        self.grid_size = 8
        self.width = self.map_pixmap.width()
        self.height = self.map_pixmap.height()
        self.grid_w = max(1, self.width // self.grid_size)
        self.grid_h = max(1, self.height // self.grid_size)

        # occupancy grid (True = blocked)
        self.occ = [[False for _ in range(self.grid_h)] for _ in range(self.grid_w)]
        self._build_occupancy_from_pixmap()

        # path visualization
        self.path_item = None
        self.path = []  # list of scene QPointF to follow
        self.path_index = 0

    """
    Convert geographic coordinates to scene coordinates.
    """
    def geo_to_scene(self, lat, lon):
        min_lat, max_lat, min_lon, max_lon = self.bbox
        width = self.map_pixmap.width()
        height = self.map_pixmap.height()

        x = (lon - min_lon) / (max_lon - min_lon) * width
        y = (max_lat - lat) / (max_lat - min_lat) * height
        return x, y
    
    """
    Convert scene coordinates to geographic coordinates.
    """
    def scene_to_geo(self, x, y):
        min_lat, max_lat, min_lon, max_lon = self.bbox
        width = self.map_pixmap.width()
        height = self.map_pixmap.height()
        lat = max_lat - (y / height) * (max_lat - min_lat)
        lon = min_lon + (x / width) * (max_lon - min_lon)
        return lat, lon

    """
    Refresh the position of the user marker on the map.
    """
    def update_marker(self):
        x,y = self.geo_to_scene(self.lat, self.lon)
        self.marker.setPos(x, y)

    """
        Build the occupancy grid from the map pixmap.
    """
    def _build_occupancy_from_pixmap(self):
        img: QImage = self.map_pixmap.toImage().convertToFormat(QImage.Format.Format_ARGB32)
        for gx in range(self.grid_w):
            for gy in range(self.grid_h):
                # sample center of cell
                sx = int((gx + 0.5) * self.grid_size)
                sy = int((gy + 0.5) * self.grid_size)
                sx = min(self.width - 1, max(0, sx))
                sy = min(self.height - 1, max(0, sy))
                col = img.pixelColor(sx, sy)

                # set color threshold for obst
                if col.alpha() > 10 and (col.red() < red_threshold or col.green() < green_threshold or col.blue() < blue_threshold):
                    self.occ[gx][gy] = True
                else:
                    self.occ[gx][gy] = False

    """
        Convert scene coordinates to grid coordinates.
    """
    def scene_to_grid(self, x, y):
        gx = int(x / self.grid_size)
        gy = int(y / self.grid_size)
        gx = min(max(gx, 0), self.grid_w - 1)
        gy = min(max(gy, 0), self.grid_h - 1)
        return gx, gy
    
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, lidar, map_path="bldg_map.png", bbox=None):
        super().__init__()
        self.setWindowTitle("RamBOTs Autonomous Navigation")

        # create map viewer as central widget
        if bbox is None:
            bbox = (40.57521493599895, 40.57590353282978, -105.08415316215739, -105.0821656452189)
        self.viewer = IndoorNavigationViewer(map_path, bbox, lidar)
        self.setCentralWidget(self.viewer)

        # create scan widget overlay as child of the viewer's viewport so it sits on top
        self.scan_widget = ScanWidget(self.viewer, size=160)
        # ensure it is visible and above the view
        self.scan_widget.setWindowFlags(self.scan_widget.windowFlags() | Qt.WindowType.SubWindow)
        self.scan_widget.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents, False)
        self.scan_widget.show()
        self._overlay_margin = 10

        # lidar worker connection
        self.worker = lidar
        self.worker.scan_ready.connect(self.scan_widget.update_scan)
        self.worker.start()

        # initial placement
        self._position_overlay()

    def _position_overlay(self):
        # position scan_widget at top-right of the viewer viewport with margin
        vp = self.viewer.viewport()
        if not vp:
            return
        vw = vp.width()
        vh = vp.height()
        sw = self.scan_widget.width()
        sx = max(0, vw - sw - self._overlay_margin)
        sy = self._overlay_margin
        # move relative to viewport coordinates
        self.scan_widget.setParent(vp)
        self.scan_widget.move(sx, sy)
        self.scan_widget.raise_()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # reposition overlay after resize
        QtCore.QTimer.singleShot(0, self._position_overlay)

    def showEvent(self, event):
        super().showEvent(event)
        QtCore.QTimer.singleShot(0, self._position_overlay)

    def closeEvent(self, event):
        try:
            self.worker.stop()
        except Exception:
            pass
        event.accept()


def __main__():
    app = QApplication(sys.argv)
    lidar = LiDAR()
    
    bbox = (40.57521493599895, 40.57590353282978, -105.08415316215739, -105.0821656452189)
    win = MainWindow(lidar, map_path="bldg_map.png", bbox=bbox)
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    __main__()