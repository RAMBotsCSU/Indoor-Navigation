from PyQt6.QtWidgets import QApplication, QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtGui import QPixmap, QBrush, QColor, QImage, QPen, QPainterPath
from PyQt6.QtCore import QTimer, QPointF
import sys, random
import heapq, math

red_threshold = 20
green_threshold = 20
blue_threshold = 20

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

        # 40.575778935543234, -105.08325487374624
        self.lat = 40.575778935543234
        self.lon = -105.08325487374624
        self.update_marker()

        # simulated gps updates
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

    def geo_to_scene(self, lat, lon):
        min_lat, max_lat, min_lon, max_lon = self.bbox
        width = self.map_pixmap.width()
        height = self.map_pixmap.height()

        # GPS to pixel coordinates
        x = (lon - min_lon) / (max_lon - min_lon) * width
        y = (max_lat - lat) / (max_lat - min_lat) * height
        return x, y

    def scene_to_geo(self, x, y):
        min_lat, max_lat, min_lon, max_lon = self.bbox
        width = self.map_pixmap.width()
        height = self.map_pixmap.height()
        lat = max_lat - (y / height) * (max_lat - min_lat)
        lon = min_lon + (x / width) * (max_lon - min_lon)
        return lat, lon

    def update_marker(self):
        x, y = self.geo_to_scene(self.lat, self.lon)
        self.marker.setPos(x, y)

    def _build_occupancy_from_pixmap(self):

        # convert svg to image to sample pixels
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

    def scene_to_grid(self, x, y):
        gx = int(x / self.grid_size)
        gy = int(y / self.grid_size)
        gx = min(max(gx, 0), self.grid_w - 1)
        gy = min(max(gy, 0), self.grid_h - 1)
        return gx, gy

    def grid_to_scene_center(self, gx, gy):
        x = (gx + 0.5) * self.grid_size
        y = (gy + 0.5) * self.grid_size
        return x, y

    def find_nearest_free(self, gx, gy, max_r=10):
        # simple BFS in increasing radius to find nearest free cell
        if not self.occ[gx][gy]:
            return gx, gy
        for r in range(1, max_r+1):
            for dx in range(-r, r+1):
                for dy in range(-r, r+1):
                    nx, ny = gx+dx, gy+dy
                    if 0 <= nx < self.grid_w and 0 <= ny < self.grid_h:
                        if not self.occ[nx][ny]:
                            return nx, ny
        return None

    def a_star(self, start, goal):
        # start, goal are (gx,gy)
        dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
        cost_dir = {(-1,0):1,(1,0):1,(0,-1):1,(0,1):1,
                    (-1,-1):math.sqrt(2),(-1,1):math.sqrt(2),(1,-1):math.sqrt(2),(1,1):math.sqrt(2)}
        open_heap = []
        heapq.heappush(open_heap, (0, start))
        came_from = {}
        gscore = {start: 0}
        def h(a,b):
            return math.hypot(a[0]-b[0], a[1]-b[1])
        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                # reconstruct
                path = []
                cur = current
                while cur != start:
                    path.append(cur)
                    cur = came_from[cur]
                path.append(start)
                path.reverse()
                return path
            for d in dirs:
                nx = current[0] + d[0]
                ny = current[1] + d[1]
                if not (0 <= nx < self.grid_w and 0 <= ny < self.grid_h):
                    continue
                if self.occ[nx][ny]:
                    continue
                tentative_g = gscore[current] + cost_dir[d]
                neighbor = (nx, ny)
                if tentative_g < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g
                    f = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))
        return None

    def draw_path(self, scene_points):
        # remove old path
        if self.path_item:
            self.scene.removeItem(self.path_item)
            self.path_item = None
        if not scene_points:
            return
        path = QPainterPath()
        first = scene_points[0]
        path.moveTo(first.x(), first.y())
        for p in scene_points[1:]:
            path.lineTo(p.x(), p.y())
        pen = QPen(QColor("blue"))
        pen.setWidth(2)
        self.path_item = self.scene.addPath(path, pen)

    def mousePressEvent(self, event):
        # set destination on click
        scene_pt: QPointF = self.mapToScene(event.position().toPoint()) if hasattr(event, "position") else self.mapToScene(event.pos())
        # in PyQt6 event.position() returns QPointF; ensure we get QPoint
        if isinstance(scene_pt, QPointF):
            sx, sy = scene_pt.x(), scene_pt.y()
        else:
            # fallback
            sx, sy = scene_pt.x(), scene_pt.y()
        gx, gy = self.scene_to_grid(sx, sy)
        res = self.find_nearest_free(gx, gy, max_r=20)
        if res is None:
            return
        goal = res
        # start from current marker position
        mx, my = self.geo_to_scene(self.lat, self.lon)
        sg = self.scene_to_grid(mx, my)
        start = self.find_nearest_free(sg[0], sg[1], max_r=20)
        if start is None:
            return
        path_cells = self.a_star(start, goal)
        if not path_cells:
            return
        # convert to scene points
        scene_points = []
        for (cx, cy) in path_cells:
            x, y = self.grid_to_scene_center(cx, cy)
            scene_points.append(QPointF(x, y))
        self.path = scene_points
        self.path_index = 0
        self.draw_path(scene_points)

    def update_position(self):
        self.lat += random.uniform(-0.000003, 0.000003)
        self.lon += random.uniform(-0.000003, 0.000003)
        self.update_marker()


if __name__ == "__main__":
    app = QApplication(sys.argv)

    # From image - (min_lat, max_lat, min_lon, max_lon)
    # Top right 40.57590353282978, -105.08415316215739
    # Bottom left 40.57521493599895, -105.0821656452189

    bbox = (40.57521493599895, 40.57590353282978, -105.08415316215739, -105.0821656452189)

    view = GPSMapViewer("bldg-map.png", bbox)
    view.show()
    sys.exit(app.exec())
