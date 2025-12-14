import asyncio
import signal
import os
import time
import argparse

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap
from PoseProvider import PoseProvider

from planner import astar
from path_follower import PathFollower

# Your ML loop should exist somewhere like this:
# It must call: pose_provider.update_ml_map_units(x_map, y_map, conf)
from ml_pose_loop import ml_pose_update_loop


async def fusion_loop(lidar, pose_provider, running_map, max_age_sec=0.25, drain_threshold=0.8):
    while True:
        ts, angle, dist = await lidar.queue.get()

        if time.monotonic() - ts > max_age_sec:
            continue

        if lidar.queue.qsize() > lidar.queue.maxsize * drain_threshold:
            while lidar.queue.qsize() > lidar.queue.maxsize * 0.5:
                try:
                    lidar.queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

        pose = pose_provider.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)


def nearest_cell(x_cm, y_cm, cell_xy_map_units, map_units_to_cm):
    best_cell = None
    best_dist = None
    for cid, (mx, my) in cell_xy_map_units.items():
        cx = mx * map_units_to_cm
        cy = my * map_units_to_cm
        d = (cx - x_cm)**2 + (cy - y_cm)**2
        if best_dist is None or d < best_dist:
            best_dist = d
            best_cell = cid
    return best_cell


async def setup(map_units_to_cm=100.0, enable_lidar=True):
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    odom = OdometryEstimator()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=200))

    pose_provider = PoseProvider(odom, map_units_to_cm=map_units_to_cm)
    pose_provider.set_mode("rssi_ml")

    lidar = None
    running_map = RunningMap(grid_size=200, cell_size_cm=5, max_distance_mm=6000)

    if enable_lidar:
        lidar = LiDAR("/dev/ttyUSB0")
        await lidar.connect()
        await asyncio.sleep(1.0)
        lidar.start(asyncio.get_running_loop())

    return controller, odom, pose_provider, lidar, running_map


async def main_async(goal_cell=60):
    controller = odom = pose_provider = lidar = running_map = None
    tasks = []

    controller, odom, pose_provider, lidar, running_map = await setup(map_units_to_cm=100.0, enable_lidar=True)

    # Start ML pose updates
    tasks.append(asyncio.create_task(ml_pose_update_loop(pose_provider)))

    # Start LiDAR fusion if enabled
    if lidar is not None:
        tasks.append(asyncio.create_task(fusion_loop(lidar, pose_provider, running_map)))

    # Wait a moment for ML to stabilize
    await asyncio.sleep(2.0)

    follower = PathFollower(controller, pose_provider)

    # Determine start from current pose
    x, y, _ = pose_provider.pose()
    start_cell = nearest_cell(x, y, follower.cell_xy_map_units, pose_provider.map_units_to_cm)

    print(f"[RSSI_ML] Start={start_cell} Goal={goal_cell}")

    path = astar(start_cell, goal_cell)
    if not path:
        print("[RSSI_ML] No path found.")
        return

    print(f"[RSSI_ML] Path length={len(path)}")

    # Follow path
    await follower.follow(path)

    # keep running map/ML unless you want to exit immediately
    await asyncio.sleep(0.5)


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser(
        description="RSSI-ML autonomous navigation"
    )
    parser.add_argument(
        "--goal",
        type=int,
        default=60,
        help="Goal cell ID (0–60)"
    )
    args = parser.parse_args()

    goal_cell = args.goal
    print(f"[RSSI_ML] Selected goal cell: {goal_cell}")
    
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        # Change goal here (or parse argv later)
        loop.run_until_complete(main_async(goal_cell=60))
        print("[RSSI_ML] done")
    except KeyboardInterrupt:
        print("\n[RSSI_ML] shutdown requested")
    finally:
        # NOTE: in this minimal version, we rely on Ctrl+C during long runs.
        # If you want full symmetric cleanup like traditional main, tell me and I’ll add it.
        loop.close()
        print("[RSSI_ML] exited")

if __name__ == "__main__":
    main()