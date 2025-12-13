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

        self.setWindowTitle("Robot Monitor")
        self.setMinimumSize(600, 700)

        self.pose_label = QLabel()
        self.pose_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        layout = QVBoxLayout(self)
        layout.addWidget(self.pose_label)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self.update)
        self._timer.start(int(1000 / update_hz))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.GlobalColor.black)

        grid = self.running_map.get_map(normalize=True)
        if grid is None:
            return

        # --- Robot pose ---
        x_cm, y_cm, theta = self.odom.pose()

        self.pose_label.setText(
            f"x={x_cm:.1f} cm   y={y_cm:.1f} cm   θ={math.degrees(theta):.1f}°"
        )

        # --- Center map on robot ---
        cx = int(self.running_map.grid_size // 2 + x_cm / self.running_map.cell_size_cm)
        cy = int(self.running_map.grid_size // 2 + y_cm / self.running_map.cell_size_cm)

        half = 50  # visible radius in cells
        x0 = max(0, cx - half)
        y0 = max(0, cy - half)
        x1 = min(grid.shape[1], cx + half)
        y1 = min(grid.shape[0], cy + half)

        view = grid[y0:y1, x0:x1]

        img = self._numpy_to_qimage(view)
        rect = self.map_rect()
        painter.drawImage(rect, img)

        # --- Draw robot dot ---
        px = rect.center().x()
        py = rect.center().y()

        painter.setBrush(QColor(255, 0, 0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(px - 4, py - 4, 8, 8)

    def map_rect(self):
        margin = 10
        top = self.pose_label.height() + margin
        size = min(
            self.width() - 2 * margin,
            self.height() - top - margin
        )
        return QRect(margin, top, size, size)

    def _numpy_to_qimage(self, grid: np.ndarray) -> QImage:
        img = (grid * 255).clip(0, 255).astype(np.uint8)
        h, w = img.shape
        return QImage(
            img.data,
            w,
            h,
            w * img.itemsize,
            QImage.Format.Format_Grayscale8
        )
