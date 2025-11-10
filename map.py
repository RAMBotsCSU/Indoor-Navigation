from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QPixmap, QBrush, QColor
from PyQt6.QtCore import QTimer, QRectF
import sys, random

class SimpleMap(QGraphicsView):
    def __init__(self, map_path, bbox):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)
        self.map_pixmap = QPixmap(map_path)
        self.scene.addPixmap(self.map_pixmap)

        self.bbox = bbox  # (min_lat, max_lat, min_lon, max_lon)
        self.marker = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.marker.setBrush(QBrush(QColor("red")))
        self.scene.addItem(self.marker)

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position)
        self.timer.start(1000)

        self.lat, self.lon = (bbox[0] + bbox[1])/2, (bbox[2] + bbox[3])/2

    def update_position(self):
        # Simulate GPS drift
        self.lat += random.uniform(-0.0005, 0.0005)
        self.lon += random.uniform(-0.0005, 0.0005)
        self.update_marker()

    def geo_to_scene(self, lat, lon):
        # Convert geo coords to scene coords (simple linear map)
        x = (lon - self.bbox[2]) / (self.bbox[3] - self.bbox[2]) * self.map_pixmap.width()
        y = (self.bbox[1] - lat) / (self.bbox[1] - self.bbox[0]) * self.map_pixmap.height()
        return x, y

    def update_marker(self):
        x, y = self.geo_to_scene(self.lat, self.lon)
        self.marker.setPos(x, y)

app = QApplication(sys.argv)
view = SimpleMap("bldg-map.png", bbox=(37.7, 37.8, -122.5, -122.3))
view.show()
sys.exit(app.exec())
