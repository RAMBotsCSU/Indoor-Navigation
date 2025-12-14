from PyQt6.QtWidgets import QWidget, QLabel, QVBoxLayout
from PyQt6.QtCore import Qt, QTimer, QRect
from PyQt6.QtGui import QPainter, QImage, QColor
import numpy as np
import math


class RobotMonitor(QWidget):
    def __init__(self, odom, running_map, update_hz=10):
        super().__init__()

        self.odom = odom
        self.running_map = running_map

        self.setWindowTitle("Robot Mapping Monitor")
        self.setMinimumSize(600, 650)

        self.pose_label = QLabel(self)
        self.pose_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout()
        layout.addWidget(self.pose_label)
        self.setLayout(layout)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / update_hz))

    # -----------------------------------------------------

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.GlobalColor.black)

        x, y, th = self.odom.pose()
        grid = self.running_map.get_local_map(x, y, size_cells=120)

        if np.max(grid) > 0:
            grid = grid / np.max(grid)
            grid = np.flipud(grid)  # flip Y for Qt

            img = self._numpy_to_qimage(grid)
            rect = self.map_rect()
            painter.drawImage(rect, img)

            self._draw_robot(painter, rect)

        self.pose_label.setText(
            f"x={x:.1f} cm   y={y:.1f} cm   θ={math.degrees(th):.1f}°"
        )

    # -----------------------------------------------------

    def map_rect(self):
        margin = 10
        top = self.pose_label.height() + margin
        size = min(
            self.width() - 2 * margin,
            self.height() - top - margin
        )
        return QRect(margin, top, size, size)

    # -----------------------------------------------------

    def _draw_robot(self, painter, rect):
        cx = rect.center().x()
        cy = rect.center().y()

        painter.setBrush(QColor(255, 0, 0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(cx - 4, cy - 4, 8, 8)

    # -----------------------------------------------------

    def _numpy_to_qimage(self, grid: np.ndarray) -> QImage:
        img = (grid * 255).astype(np.uint8)
        h, w = img.shape
        return QImage(
            img.data,
            w,
            h,
            w,
            QImage.Format.Format_Grayscale8
        )
