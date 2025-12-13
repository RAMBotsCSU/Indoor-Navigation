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
    """Example motion script: move forward 10 cm every step."""
    for i in range(100):
        print(f"Moving forward 10 cm (step {i+1}/100)")
        success = await controller.forward_cm(10.0)
        print(f"Movement {'succeeded' if success else 'timed out'}")
        await asyncio.sleep(0.1)


async def async_setup():
    """Async initialization for LiDAR, odometry, mapping."""
    # --- Motion Controller (moves motors) ---
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    # --- Odometry Estimator (tracks pose continuously) ---
    odom_estimator = OdometryEstimator()
    await odom_estimator.connect()
    asyncio.create_task(odom_estimator.start(rate_hz=200))

    # --- LiDAR ---
    lidar = LiDAR('/dev/ttyUSB0')
    lidar.start(asyncio.get_running_loop())

    # --- Mapping ---
    running_map = RunningMap(grid_size=200, cell_size_cm=5, max_distance_mm=6000)

    # --- Start fusion ---
    asyncio.create_task(fusion_loop(lidar, odom_estimator, running_map))

    return controller, odom_estimator, lidar, running_map


def main():
    """Qt must run in main thread; asyncio in background."""
    app = QApplication(sys.argv)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # --- Setup async components ---
    controller, odom_estimator, lidar, running_map = loop.run_until_complete(async_setup())

    # --- UI ---
    ui = RobotMonitor(odom_estimator, running_map)
    ui.show()

    # --- Motion script runs in background ---
    asyncio.create_task(motion_script(controller))

    try:
        sys.exit(app.exec())
    finally:
        print("Shutting down...")
        lidar.stop()
        controller.stop()
        odom_estimator.stop()
        loop.stop()
        loop.close()


if __name__ == "__main__":
    main()
