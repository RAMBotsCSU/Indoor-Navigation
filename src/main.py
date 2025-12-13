import sys
import asyncio
from PyQt6.QtWidgets import QApplication

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap
from RobotMonitor import RobotMonitor


async def fusion_loop(lidar, odom_estimator, running_map):
    """Continuously fuse LiDAR points with timestamped odometry."""
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom_estimator.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)


async def motion_script(controller: RobotController):
    """Example motion: move forward + turn repeatedly."""
    try:
        for i in range(50):
            print(f"Step {i+1}: moving forward 10 cm")
            await controller.forward_cm(10.0)
            await asyncio.sleep(0.1)
            print(f"Step {i+1}: turning 15 deg CCW")
            await controller.turn_deg(15)
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        print("Motion script cancelled")


async def async_setup():
    """Initialize controller, odometry, LiDAR, and map."""
    # --- Motion Controller ---
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    # --- Odometry Estimator ---
    odom_estimator = OdometryEstimator()
    await odom_estimator.connect()
    asyncio.create_task(odom_estimator.start(rate_hz=200))

    # --- LiDAR ---
    lidar = LiDAR('/dev/ttyUSB0')
    await lidar.connect()  # Make sure LiDAR has async connect
    await asyncio.sleep(1.0)  # Give LiDAR time to stabilize
    lidar.start(asyncio.get_running_loop())

    # --- Running Map ---
    running_map = RunningMap(grid_size=200, cell_size_cm=5, max_distance_mm=6000)

    # --- Start fusion loop ---
    asyncio.create_task(fusion_loop(lidar, odom_estimator, running_map))

    return controller, odom_estimator, lidar, running_map


def main():
    app = QApplication(sys.argv)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # Async setup
    controller, odom_estimator, lidar, running_map = loop.run_until_complete(async_setup())

    # --- UI ---
    ui = RobotMonitor(odom_estimator, running_map)
    ui.show()

    # --- Motion task (create it in a running loop context) ---
    async def start_motion():
        return asyncio.create_task(motion_script(controller))
    
    motion_task = loop.run_until_complete(start_motion())
    
    try:
        sys.exit(app.exec())
    finally:
        # --- Cleanup ---
        print("Shutting down...")
        motion_task.cancel()
        try:
            loop.run_until_complete(motion_task)
        except asyncio.CancelledError:
            pass

        loop.run_until_complete(lidar.stop())
        loop.run_until_complete(controller.stop())
        odom_estimator.stop()
        loop.stop()
        loop.close()


if __name__ == "__main__":
    main()
