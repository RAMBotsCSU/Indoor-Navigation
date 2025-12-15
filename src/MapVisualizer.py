import sys
from PyQt6.QtWidgets import (QMainWindow, QVBoxLayout, QHBoxLayout, 
                             QWidget, QSlider, QLabel, QComboBox)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QFont
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class MapCanvas(FigureCanvas):
    def __init__(self, parent=None):
        self.fig = Figure(figsize=(8, 8), dpi=100)
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)
        self.setParent(parent)
        self.im = None
        
    def update_map(self, grid_data, vmin=0, vmax=1, cmap="hot"):
        """Update the displayed map."""
        self.ax.clear()
        self.im = self.ax.imshow(grid_data, cmap=cmap, origin="lower", vmin=vmin, vmax=vmax)
        self.ax.set_title("Occupancy Grid Map")
        self.ax.axis('off')
        self.fig.tight_layout(pad=0)
        self.draw()

class MapVisualizerWindow(QMainWindow):
    def __init__(self, running_map):
        super().__init__()
        self.running_map = running_map
        self.setWindowTitle("Running Map Visualizer")
        self.setGeometry(100, 100, 1000, 900)
        
        # Main widget
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # Layout
        layout = QVBoxLayout(main_widget)
        
        # Canvas
        self.canvas = MapCanvas(main_widget)
        layout.addWidget(self.canvas)
        
        # Control panel
        control_layout = QHBoxLayout()
        
        # Colormap selector
        control_layout.addWidget(QLabel("Colormap:"))
        self.cmap_combo = QComboBox()
        self.cmap_combo.addItems(["hot", "viridis", "plasma", "cool", "gray"])
        self.cmap_combo.currentTextChanged.connect(self.on_update)
        control_layout.addWidget(self.cmap_combo)
        
        # Threshold slider (for binary visualization)
        control_layout.addWidget(QLabel("Binary Threshold:"))
        self.threshold_slider = QSlider(Qt.Orientation.Horizontal)
        self.threshold_slider.setMinimum(0)
        self.threshold_slider.setMaximum(100)
        self.threshold_slider.setValue(60)
        self.threshold_slider.setMaximumWidth(150)
        self.threshold_slider.valueChanged.connect(self.on_update)
        control_layout.addWidget(self.threshold_slider)
        
        # Threshold value label
        self.threshold_label = QLabel("0.60")
        self.threshold_label.setMaximumWidth(50)
        control_layout.addWidget(self.threshold_label)
        
        # View mode selector
        control_layout.addWidget(QLabel("View:"))
        self.view_combo = QComboBox()
        self.view_combo.addItems(["Probability", "Binary"])
        self.view_combo.currentTextChanged.connect(self.on_update)
        control_layout.addWidget(self.view_combo)
        
        control_layout.addStretch()
        layout.addLayout(control_layout)
        
        # Update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.on_update)
        self.update_timer.start(500)  # Update every 500ms
        
    def on_update(self):
        """Update the displayed map."""
        grid_data = self.running_map.grid.copy()
        threshold = self.threshold_slider.value() / 100.0
        self.threshold_label.setText(f"{threshold:.2f}")
        cmap = self.cmap_combo.currentText()
        view_mode = self.view_combo.currentText()
        
        if view_mode == "Binary":
            grid_data = (grid_data >= threshold).astype(float)
            self.canvas.update_map(grid_data, vmin=0, vmax=1, cmap="gray_r")
        else:
            self.canvas.update_map(grid_data, vmin=0, vmax=1, cmap=cmap)

def run_visualizer(running_map):
    """Run the visualizer in a separate Qt event loop."""
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    window = MapVisualizerWindow(running_map)
    window.show()
    sys.exit(app.exec())
