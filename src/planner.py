import heapq
import json
import os
import math

# -------------------------------------------------
# Load map + neighbors once
# -------------------------------------------------
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
INCLUDE_DIR = os.path.join(BASE_DIR, "..", "include")

with open(os.path.join(INCLUDE_DIR, "classmap.json")) as f:
    _classmap = json.load(f)["cells"]

with open(os.path.join(INCLUDE_DIR, "neighbors.json")) as f:
    _neighbors = json.load(f)["neighbors"]

# Convert keys
CLASS_POS = {
    int(k): (v["x"], v["y"]) for k, v in _classmap.items()
}
NEIGHBORS = {
    int(k): v for k, v in _neighbors.items()
}

# -------------------------------------------------
# Heuristic: straight-line distance
# -------------------------------------------------
def heuristic(a: int, b: int) -> float:
    ax, ay = CLASS_POS[a]
    bx, by = CLASS_POS[b]
    return math.hypot(bx - ax, by - ay)

# -------------------------------------------------
# A* search
# -------------------------------------------------
def astar(start: int, goal: int):
    """
    Returns list of cell IDs from start → goal.
    """
    if start == goal:
        return [start]

    open_set = []
    heapq.heappush(open_set, (0, start))

    came_from = {}
    g_score = {start: 0.0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        for nb in NEIGHBORS.get(current, []):
            tentative = g_score[current] + heuristic(current, nb)

            if nb not in g_score or tentative < g_score[nb]:
                came_from[nb] = current
                g_score[nb] = tentative
                f = tentative + heuristic(nb, goal)
                heapq.heappush(open_set, (f, nb))

    return None