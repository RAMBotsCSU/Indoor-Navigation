import sys
import csv
from math import cos, sin, pi, sqrt
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor
from PyQt6.QtCore import Qt, QTimer


CSV_PATH = "include/lidar_data_old.csv"   
NOISE_THRESHOLD = 200        


class LidarViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR CSV Viewer")
        self.resize(800, 800)

        self.label = QLabel(self)
        self.label.setFixedSize(800, 800)
        self.setCentralWidget(self.label)

        # Load CSV
        self.scans = self.load_csv(CSV_PATH)
        self.index = 0

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)  # ~20 FPS

    def load_csv(self, path):
        scans = []
        with open(path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                row = row[:360]  # ensure 360 angles
                try:
                    scans.append([float(x) for x in row])
                except:
                    pass
        return scans

    def update_frame(self):
        if not self.scans:
            return

        scan = self.scans[self.index]
        self.index = (self.index + 1) % len(self.scans)
        self.label.setPixmap(self.plot_scan(scan))

    def plot_scan(self, scan):
        """Draw LiDAR as connected lines with noise rejection."""
        pix = QPixmap(800, 800)
        pix.fill(Qt.GlobalColor.black)

        painter = QPainter(pix)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        pen = QPen(QColor("white"))
        pen.setWidth(2)
        painter.setPen(pen)

        cx, cy = 400, 400
        scale = 0.1

        last_x = None
        last_y = None
        last_dist = None

        for angle_deg, dist in enumerate(scan):
            if dist <= 0:
                last_x = None
                last_y = None
                last_dist = None
                continue

            rad = angle_deg * pi / 180.0
            x = cx + dist * cos(rad) * scale
            y = cy + dist * sin(rad) * scale

            if last_x is not None:
                # noise rejection: skip if sudden huge jump
                if abs(dist - last_dist) < NOISE_THRESHOLD:
                    painter.drawLine(int(last_x), int(last_y), int(x), int(y))

            last_x = x
            last_y = y
            last_dist = dist

        painter.end()
        return pix


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = LidarViewer()
    viewer.show()
    sys.exit(app.exec())
