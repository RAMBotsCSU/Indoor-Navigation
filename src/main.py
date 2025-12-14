import sys
import asyncio
import os
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap
from RobotMonitor import RobotMonitor


async def fusion_loop(lidar, odom, running_map):
    """Fuse LiDAR points with timestamped odometry."""
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom.interpolate(ts)
        if pose is not None:
            running_map.integrate_point(angle, dist, pose)


async def motion_script(controller):
    """Simple autonomous motion."""
    try:
        while True:
            await controller.forward_cm(10.0)
            await asyncio.sleep(0.1)
            await controller.turn_deg(15)
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass


async def async_setup(loop):
    # --- Motion ---
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    # --- Odometry ---
    odom = OdometryEstimator()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=200))

    # --- LiDAR ---
    lidar = LiDAR("/dev/ttyUSB0")
    lidar.connect()
    await asyncio.sleep(1.0)
    lidar.start(loop)

    # --- Map ---
    running_map = RunningMap(
        grid_size=200,
        cell_size_cm=5,
        max_distance_mm=6000
    )

    # --- Background tasks ---
    asyncio.create_task(fusion_loop(lidar, odom, running_map))

    return controller, odom, lidar, running_map


def main():
    app = QApplication(sys.argv)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    controller, odom, lidar, running_map = loop.run_until_complete(
        async_setup(loop)
    )

    # --- UI ---
    ui = RobotMonitor(odom, running_map)
    ui.show()

    # --- Motion ---
    motion_task = loop.create_task(motion_script(controller))

    # --- Integrate asyncio with Qt ---
    def step_asyncio():
        loop.call_soon(loop.stop)
        loop.run_forever()

    timer = QTimer()
    timer.timeout.connect(step_asyncio)
    timer.start(10)

    try:
        sys.exit(app.exec())
    finally:
        print("Shutting down...")
        timer.stop()

        motion_task.cancel()
        try:
            loop.run_until_complete(motion_task)
        except asyncio.CancelledError:
            pass

        lidar.stop()
        loop.run_until_complete(controller.stop())
        odom.stop()

        os.makedirs("outputs", exist_ok=True)
        running_map.save_heatmap("outputs/map_final.png")

        loop.close()


if __name__ == "__main__":
    main()
