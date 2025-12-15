import asyncio
import signal
import os
import time
import argparse

from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from PoseProvider import PoseProvider
from NavToolkit import NavToolkit  # live RSSI + ML pose updates and A*

from PathFollower import PathFollower


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


async def setup(map_units_to_cm=100.0):
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    odom = OdometryEstimator()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=50))

    pose_provider = PoseProvider(odom, map_units_to_cm=map_units_to_cm)
    pose_provider.set_mode("rssi_ml")

    return controller, odom, pose_provider


# Simple LiDAR-based stop: if anything in front is closer than stop_distance_cm, stop motors.
async def lidar_guard(controller: RobotController, odom: OdometryEstimator, stop_distance_cm=30.0, fov_deg=180, check_hz=10.0, obstacle_event: asyncio.Event | None = None):
    period = 1.0 / check_hz
    half = fov_deg / 2.0
    while True:
        try:
            scan = getattr(odom, "latest_scan", None)  # expected [(angle_deg, dist_cm), ...]
            if scan:
                front = [d for a, d in scan if -half <= a <= half]
                if front and min(front) <= stop_distance_cm:
                    try:
                        await controller.stop()
                    except Exception:
                        pass
                    if obstacle_event:
                        obstacle_event.set()
            await asyncio.sleep(period)
        except asyncio.CancelledError:
            break
        except Exception:
            await asyncio.sleep(period)


async def main_async(goal_cell=60):
    controller = odom = pose_provider = None
    tasks = []
    controller, odom, pose_provider = await setup(map_units_to_cm=100.0)

    # Start live RSSI to ML pose updates
    ml_task = asyncio.create_task(
        NavToolkit.ml_pose_update_loop(
            pose_provider=pose_provider,
            interface="wlan0",
            scan_hz=1.0,
            sudo_scan=True,
            missing_dbm=-100.0,
        )
    )
    tasks.append(ml_task)

    await asyncio.sleep(2.0)  # allow ML to stabilize

    follower = PathFollower(controller, pose_provider)
    x, y, _ = pose_provider.pose()
    start_cell = nearest_cell(x, y, follower.cell_xy_map_units, pose_provider.map_units_to_cm)

    print(f"[RSSI_ML] Start={start_cell} Goal={goal_cell}")
    path = NavToolkit.astar(start_cell, goal_cell)
    if not path:
        print("[RSSI_ML] No path found.")
    else:
        print(f"[RSSI_ML] Path length={len(path)}")
        # Start LiDAR safety guard with obstacle event
        obstacle_event = asyncio.Event()
        tasks.append(asyncio.create_task(lidar_guard(controller, odom, stop_distance_cm=30.0, fov_deg=180, check_hz=10.0, obstacle_event=obstacle_event)))
        try:
            await follower.follow(path)
        except Exception:
            pass

        # If an obstacle was detected, replan from current pose
        if obstacle_event.is_set():
            obstacle_event.clear()
            x, y, _ = pose_provider.pose()
            current_cell = nearest_cell(x, y, follower.cell_xy_map_units, pose_provider.map_units_to_cm)
            try:
                await controller.stop()
            except Exception:
                pass
            # Prefer a safer planner if available
            if hasattr(NavToolkit, "astar_safe"):
                new_path = NavToolkit.astar_safe(current_cell, goal_cell)
            else:
                new_path = NavToolkit.astar(current_cell, goal_cell)
            if new_path:
                print(f"[RSSI_ML] Replanned path length={len(new_path)}")
                await follower.follow(new_path)
            else:
                print("[RSSI_ML] Replan failed; stopping.")
        await asyncio.sleep(0.5)

    for t in tasks:
        t.cancel()
    await asyncio.gather(*tasks, return_exceptions=True)


def main():
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    parser = argparse.ArgumentParser(
        description="RSSI-ML autonomous navigation"
    )
    parser.add_argument(
        "--goal",
        type=int,
        default=60,
        help="Goal cell ID (0-60)"
    )
    args = parser.parse_args()

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    try:
        loop.run_until_complete(main_async(goal_cell=args.goal))
        print("[RSSI_ML] done")
    except KeyboardInterrupt:
        print("\n[RSSI_ML] shutdown requested")
    finally:
        loop.close()
        print("[RSSI_ML] exited")


if __name__ == "__main__":
    main()