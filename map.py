from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QPixmap, QBrush, QColor
from PyQt6.QtCore import QTimer
import sys, random

class GPSMapViewer(QGraphicsView):
    def __init__(self, map_path, bbox):
        super().__init__()
        self.scene = QGraphicsScene()
        self.setScene(self.scene)

        # import map image
        self.map_pixmap = QPixmap(map_path)
        self.scene.addPixmap(self.map_pixmap)

        # bounding box
        # (min_lat, max_lat, min_lon, max_lon)
        self.bbox = bbox

        # GPS marker
        self.marker = QGraphicsEllipseItem(-5, -5, 10, 10)
        self.marker.setBrush(QBrush(QColor("red")))
        self.scene.addItem(self.marker)

        # 40.57579702201656, -105.08348366394911
        self.lat = 40.57579702201656
        self.lon = -105.08348366394911
        self.update_marker()

        # simulated gps updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_position)
        self.timer.start(1000)

    def geo_to_scene(self, lat, lon):
        min_lat, max_lat, min_lon, max_lon = self.bbox
        width = self.map_pixmap.width()
        height = self.map_pixmap.height()

        # GPS to pixel coordinates
        x = (lon - min_lon) / (max_lon - min_lon) * width
        y = (max_lat - lat) / (max_lat - min_lat) * height
        return x, y

    def update_marker(self):
        x, y = self.geo_to_scene(self.lat, self.lon)
        self.marker.setPos(x, y)

    def update_position(self):
        # simulate small random movement
        self.lat += random.uniform(-0.000003, 0.000003)
        self.lon += random.uniform(-0.000003, 0.000003)
        self.update_marker()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # From image - (min_lat, max_lat, min_lon, max_lon)
    # Top right 40.575920, -105.084196
    # Bottom left 40.575243, -105.082178

    bbox = (40.575243, 40.575920, -105.084196, -105.082178)

    view = GPSMapViewer("bldg-map.png", bbox)
    view.show()
    sys.exit(app.exec())
