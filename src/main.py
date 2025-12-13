import sys
import asyncio
import os
from PyQt6.QtWidgets import QApplication

from LiDAR import LiDAR
from Odometry import Odometry
from RunningMap import RunningMap
from LiDAR_UI import LiDAR_UI


async def fusion_loop(lidar, odom, running_map):
    """
    Continuously fuse LiDAR points with timestamped odometry.
    """
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)


async def async_setup():
    """
    Async initialization (NO Qt here).
    """
    # --- Odometry ---
    odom = Odometry()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=200))

    # --- LiDAR ---
    lidar = LiDAR('/dev/ttyUSB0')
    lidar.start(asyncio.get_running_loop())

    # --- Mapping ---
    running_map = RunningMap(
        grid_size=200,
        cell_size_cm=5,
        max_distance_mm=6000
    )

    # --- Fusion loop ---
    asyncio.create_task(fusion_loop(lidar, odom, running_map))

    return lidar, odom, running_map


def main():
    """
    Qt MUST run in main thread.
    asyncio runs in background.
    """
    # --- Qt App ---
    app = QApplication(sys.argv)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    lidar, odom, running_map = loop.run_until_complete(async_setup())

    # --- UI ---
    ui = LiDAR_UI(odom, running_map)
    ui.show()

    try:
        sys.exit(app.exec())
    finally:
        # --- Cleanup ---
        print("Shutting down...")
        lidar.stop()
        odom.stop()
        loop.stop()
        loop.close()


if __name__ == "__main__":
    main()
