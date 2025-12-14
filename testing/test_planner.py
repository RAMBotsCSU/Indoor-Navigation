import json, math, heapq
import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

with open(os.path.join(BASE_DIR, "classmap.json")) as f:
    cm = json.load(f)

with open(os.path.join(BASE_DIR, "neighbors.json")) as f:
    nb = json.load(f)



xy = {int(k): (v["x"], v["y"]) for k, v in cm["cells"].items()}
neighbors = {int(k): [int(x) for x in v] for k, v in nb["neighbors"].items()}

def dist(a,b):
    ax, ay = xy[a]
    bx, by = xy[b]
    return math.hypot(bx-ax, by-ay)

def astar(s,g):
    pq = [(0,s)]
    came = {s: None}
    cost = {s: 0.0}
    while pq:
        _, cur = heapq.heappop(pq)
        if cur == g: break
        for nxt in neighbors.get(cur, []):
            cand = cost[cur] + dist(cur,nxt)
            if nxt not in cost or cand < cost[nxt]:
                cost[nxt] = cand
                heapq.heappush(pq, (cand + dist(nxt,g), nxt))
                came[nxt] = cur
    if g not in came:
        return None
    path=[]
    n=g
    while n is not None:
        path.append(n); n=came[n]
    return list(reversed(path))

path = astar(0, 60)
print("Path 0->60:", path)
print("Path length:", len(path))