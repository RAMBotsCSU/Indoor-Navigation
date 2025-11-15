import sys
import csv
from math import cos, sin, pi
from PyQt6.QtWidgets import QApplication, QLabel, QMainWindow
from PyQt6.QtGui import QPixmap, QPainter, QPen, QColor, QBrush
from PyQt6.QtCore import Qt, QTimer


CSV_PATH = "lidar_data_old.csv"   # <— change to your CSV file


class LidarViewer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR CSV Viewer")
        self.resize(800, 800)

        self.label = QLabel(self)
        self.label.setFixedSize(800, 800)
        self.setCentralWidget(self.label)

        # Load entire CSV into memory
        self.scans = self.load_csv(CSV_PATH)
        self.index = 0

        # update ~20 fps
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(50)

    def load_csv(self, path):
        scans = []
        with open(path, "r") as f:
            reader = csv.reader(f)
            for row in reader:
                row = row[:360]  # keep 360 cols
                try:
                    scans.append([float(x) for x in row])
                except:
                    pass  # skip malformed
        return scans

    def update_frame(self):
        if not self.scans:
            return

        scan = self.scans[self.index]
        self.index = (self.index + 1) % len(self.scans)

        pixmap = self.plot_scan(scan)
        self.label.setPixmap(pixmap)

    def plot_scan(self, scan):
        """Draw 360 LiDAR points."""
        pix = QPixmap(800, 800)
        pix.fill(Qt.GlobalColor.black)   # <-- FIXED

        painter = QPainter(pix)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        pen = QPen(QColor("white"))
        painter.setPen(pen)
        painter.setBrush(QBrush(QColor("white")))

        cx, cy = 400, 400
        scale = 0.1

        for angle_deg, dist in enumerate(scan):
            if dist <= 0:
                continue

            rad = angle_deg * pi / 180
            x = cx + dist * cos(rad) * scale
            y = cy + dist * sin(rad) * scale

            painter.drawEllipse(int(x)-2, int(y)-2, 4, 4)

        painter.end()
        return pix


if __name__ == "__main__":
    app = QApplication(sys.argv)
    viewer = LidarViewer()
    viewer.show()
    sys.exit(app.exec())
