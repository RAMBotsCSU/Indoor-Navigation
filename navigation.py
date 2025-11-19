from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QPixmap, QBrush, QColor, QImage, QPen, QPainterPath
from PyQt6.QtCore import QTimer, QPointF
from adafruit_rplidar import RPLidar
import sys, random
import heapq, math, os

red_threshold = 20
green_threshold = 20
blue_threshold = 20

PORT_NAME = '/dev/ttyUSB0' # usb device name
max_distance = 0  # max LiDAR distance in mm

class LiDAR():
    def __init__(self):
        os.putenv('SDL_FBDEV', '/dev/fb1')
        self.lidar = RPLidar(None, PORT_NAME, timeout=7)
        self.scan_data = [0]*360
        lidar_data=[]
        print(self.lidar.info)
        


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
    

def __main__():
    app = QApplication(sys.argv)
    lidar = LiDAR()
    
    # viewer = IndoorNavigationViewer("indoor_map.png", (37.4219999, 37.4229999, -122.0840575, -122.0830575), lidar)
    # viewer.setWindowTitle("Indoor Navigation Viewer")
    # viewer.resize(800, 600)
    # viewer.show()
    # sys.exit(app.exec())