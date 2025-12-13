from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QPainter, QImage
import numpy as np
import math

class RobotMonitor(QWidget):
    def __init__(self, odom, running_map, update_hz=10):
        super().__init__()

        self.odom = odom
        self.running_map = running_map

        self.setWindowTitle("LiDAR UI")
        self.setMinimumSize(500, 600)

        self.pose_label = QLabel(self)
        self.pose_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(self.pose_label)
        self.setLayout(layout)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(int(1000 / update_hz))

    def paintEvent(self, event):
        painter = QPainter(self)

        # draw background
        painter.fillRect(self.rect(), Qt.GlobalColor.black)

        # draw map
        grid = self.running_map.get_map(normalize=True)
        if grid is not None:
            img = self._numpy_to_qimage(grid)
            painter.drawImage(self.map_rect(), img)

        # draw pose
        x, y, th = self.odom.pose()
        self.pose_label.setText(
            f"x = {x:.2f} cm   y = {y:.2f} cm   θ = {math.degrees(th):.1f}°"
        )

    def map_rect(self):
        """rectangle for map"""
        margin = 10
        top = self.pose_label.height() + margin
        size = min(
            self.width() - 2 * margin,
            self.height() - top - margin
        )
        return QRect(
            margin,
            top,
            size,
            size
        )

    def _numpy_to_qimage(self, grid: np.ndarray) -> QImage:
        """
        convert normalized 2D numpy array to grayscale QImage.
        """
        img = (grid * 255).clip(0, 255).astype(np.uint8)
        h, w = img.shape
        return QImage(
            img.data,
            w,
            h,
            w,
            QImage.Format.Format_Grayscale8
        )
