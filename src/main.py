import sys
import asyncio
from PyQt6.QtWidgets import QApplication

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap
from RobotMonitor import RobotMonitor  # UI class showing map and robot pose


async def fusion_loop(lidar, odom_estimator, running_map):
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom_estimator.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)


async def motion_script(controller: RobotController):
    for i in range(50):
        print(f"Step {i+1}: moving forward 10 cm")
        success = await controller.forward_cm(10.0)
        print(f"Forward move {'succeeded' if success else 'timed out'}")
        await asyncio.sleep(0.1)

        print(f"Step {i+1}: turning 15 deg CCW")
        success = await controller.turn_deg(15)
        print(f"Turn {'succeeded' if success else 'timed out'}")
        await asyncio.sleep(0.1)


async def async_setup():

    controller = RobotController()
    await controller.connect()
    await controller.enable()

    odom_estimator = OdometryEstimator()
    await odom_estimator.connect()
    asyncio.create_task(odom_estimator.start(rate_hz=200))  # continuous updates


    lidar = LiDAR('/dev/ttyUSB0')
    lidar.start(asyncio.get_running_loop())

    running_map = RunningMap(
        grid_size=200,
        cell_size_cm=5,
        max_distance_mm=6000
    )

    asyncio.create_task(fusion_loop(lidar, odom_estimator, running_map))

    return controller, odom_estimator, lidar, running_map


def main():
    """Qt main thread; asyncio runs in background."""
    app = QApplication(sys.argv)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    controller, odom_estimator, lidar, running_map = loop.run_until_complete(async_setup())

    ui = RobotMonitor(odom_estimator, running_map)
    ui.show()

    asyncio.create_task(motion_script(controller))

    try:
        sys.exit(app.exec())
    finally:
        print("Shutting down...")
        lidar.stop()
        asyncio.run(controller.stop())
        odom_estimator.stop()
        loop.stop()
        loop.close()


if __name__ == "__main__":
    main()
