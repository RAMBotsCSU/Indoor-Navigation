import asyncio
import signal
import os
import time
import argparse
import traceback

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
    controller = RobotController(update_rate_hz=50)
    await controller.connect()
    await controller.enable()

    odom = OdometryEstimator()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=200))

    pose_provider = PoseProvider(odom, map_units_to_cm=map_units_to_cm)
    pose_provider.set_mode("rssi_ml")

    return controller, odom, pose_provider


# Simple LiDAR-based stop: if anything in front is closer than stop_distance_cm, stop motors.
async def lidar_guard(controller: RobotController, odom: OdometryEstimator, stop_distance_cm=30.0, fov_deg=180, check_hz=10.0, obstacle_event: asyncio.Event | None = None):
    period = 1.0 / check_hz
    half = fov_deg / 2.0
    stopping = False  # Prevent redundant stop calls
    while True:
        try:
            scan = getattr(odom, "latest_scan", None)
            if scan:
                front = [d for a, d in scan if -half <= a <= half]
                if front and min(front) <= stop_distance_cm:
                    if not stopping:
                        stopping = True
                        try:
                            await controller.stop()
                        except Exception as e:
                            print(f"[LIDAR] Stop failed: {e}")
                        if obstacle_event:
                            obstacle_event.set()
                else:
                    stopping = False
            await asyncio.sleep(period)
        except asyncio.CancelledError:
            break
        except Exception as e:
            print(f"[LIDAR] Error: {e}")
            await asyncio.sleep(period)


async def main_async(goal_cell=60):
    controller = odom = pose_provider = None
    tasks = []
    
    try:
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

        await asyncio.sleep(2.0)

        follower = PathFollower(controller, pose_provider)
        
        x, y, _ = pose_provider.pose()
        start_cell = nearest_cell(x, y, follower.cell_xy_map_units, pose_provider.map_units_to_cm)
        
        if start_cell is None:
            print("[RSSI_ML] Error: Could not determine starting cell")
            return

        print(f"[RSSI_ML] Start={start_cell} Goal={goal_cell}")
        path = NavToolkit.astar(start_cell, goal_cell)
        if not path:
            print("[RSSI_ML] No path found.")
            return
            
        print(f"[RSSI_ML] Path length={len(path)}")
        
        # Start LiDAR safety guard with obstacle event
        obstacle_event = asyncio.Event()
        guard_task = asyncio.create_task(lidar_guard(controller, odom, stop_distance_cm=30.0, fov_deg=180, check_hz=10.0, obstacle_event=obstacle_event))
        tasks.append(guard_task)
        
        # Main navigation loop with obstacle replanning
        max_replans = 3
        replan_count = 0
        current_path = path
        
        while current_path and replan_count <= max_replans:
            try:
                await follower.follow(current_path)
                # Successfully completed path
                break
            except Exception as e:
                print(f"[RSSI_ML] Path following error: {e}")
                traceback.print_exc()

            # Check if obstacle was detected
            if obstacle_event.is_set():
                obstacle_event.clear()
                replan_count += 1
                print(f"[RSSI_ML] Obstacle detected, replanning (attempt {replan_count}/{max_replans})")
                
                x, y, _ = pose_provider.pose()
                current_cell = nearest_cell(x, y, follower.cell_xy_map_units, pose_provider.map_units_to_cm)
                
                if current_cell is None:
                    print("[RSSI_ML] Error: Could not determine current cell")
                    break
                
                try:
                    await controller.stop()
                except Exception as e:
                    print(f"[RSSI_ML] Stop failed: {e}")
                
                await asyncio.sleep(0.5)  # Brief pause before replanning
                
                # Prefer safer planner if available
                if hasattr(NavToolkit, "astar_safe"):
                    current_path = NavToolkit.astar_safe(current_cell, goal_cell)
                else:
                    current_path = NavToolkit.astar(current_cell, goal_cell)
                
                if current_path:
                    print(f"[RSSI_ML] Replanned path length={len(current_path)}")
                else:
                    print("[RSSI_ML] Replan failed; stopping.")
                    break
            else:
                # Path following failed without obstacle detection
                break
        
        if replan_count > max_replans:
            print(f"[RSSI_ML] Max replanning attempts ({max_replans}) reached")
        
        await asyncio.sleep(0.5)

    except Exception as e:
        print(f"[RSSI_ML] Fatal error: {e}")
        traceback.print_exc()
    finally:
        # Cancel all background tasks
        for t in tasks:
            if not t.done():
                t.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        
        # Clean up resources
        try:
            if controller:
                await controller.stop()
                await controller.disable()
                # Add disconnect if available
                if hasattr(controller, 'disconnect'):
                    await controller.disconnect()
        except Exception as e:
            print(f"[RSSI_ML] Controller cleanup error: {e}")
        
        try:
            if odom and hasattr(odom, 'stop'):
                await odom.stop()
            if odom and hasattr(odom, 'disconnect'):
                await odom.disconnect()
        except Exception as e:
            print(f"[RSSI_ML] Odometry cleanup error: {e}")


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