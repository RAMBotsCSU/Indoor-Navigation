import sys
import asyncio
import os
import signal
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap
from RobotMonitor import RobotMonitor


# ---------------------------------------------------------
# Async background tasks
# ---------------------------------------------------------

async def fusion_loop(lidar, odom, running_map):
    """Fuse LiDAR points with interpolated odometry."""
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)


async def motion_script(controller):
    """Simple autonomous motion script."""
    try:
        while True:
            await controller.forward_cm(10.0)
            await asyncio.sleep(0.1)
            await controller.turn_deg(15.0)
            await asyncio.sleep(0.1)
    except asyncio.CancelledError:
        pass


async def autosave_heatmap(running_map, interval_s=5, out_dir="outputs"):
    os.makedirs(out_dir, exist_ok=True)
    idx = 0
    while True:
        idx += 1
        path = os.path.join(out_dir, f"map_{idx:04d}.png")
        running_map.save_heatmap(path)
        await asyncio.sleep(interval_s)


# ---------------------------------------------------------
# Async initialization
# ---------------------------------------------------------

async def async_setup(loop):
    # --- Motion Controller ---
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    # --- Odometry ---
    odom = OdometryEstimator()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=200))

    # --- LiDAR ---
    lidar = LiDAR("/dev/ttyUSB0")
    await lidar.connect()
    lidar.start(loop)

    # --- Map ---
    running_map = RunningMap(
        grid_size=200,
        cell_size_cm=5,
        max_distance_mm=6000
    )

    # --- Background tasks ---
    tasks = [
        asyncio.create_task(fusion_loop(lidar, odom, running_map)),
        asyncio.create_task(motion_script(controller)),
        asyncio.create_task(autosave_heatmap(running_map)),
    ]

    return controller, odom, lidar, running_map, tasks


# ---------------------------------------------------------
# Main (Qt thread)
# ---------------------------------------------------------

def main():
    # --- Qt App (MUST be main thread) ---
    app = QApplication(sys.argv)

    # --- Asyncio loop ---
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # --- Setup ---
    controller, odom, lidar, running_map, tasks = loop.run_until_complete(
        async_setup(loop)
    )

    # --- UI ---
    ui = RobotMonitor(odom, running_map)
    ui.show()

    # --- Drive asyncio loop from Qt ---
    def pump_asyncio():
        loop.call_soon(loop.stop)
        loop.run_forever()

    timer = QTimer()
    timer.timeout.connect(pump_asyncio)
    timer.start(10)  # 100 Hz event pumping

    # --- Clean shutdown ---
    def shutdown():
        print("Shutting down...")
        timer.stop()

        for t in tasks:
            t.cancel()

        loop.run_until_complete(asyncio.gather(*tasks, return_exceptions=True))

        loop.run_until_complete(lidar.stop())
        loop.run_until_complete(controller.stop())
        odom.stop()

        final_path = os.path.join("outputs", "map_final.png")
        running_map.save_heatmap(final_path)
        print(f"[Saved] {final_path}")

        loop.stop()
        loop.close()

    app.aboutToQuit.connect(shutdown)
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    sys.exit(app.exec())


# ---------------------------------------------------------
# Entry
# ---------------------------------------------------------

if __name__ == "__main__":
    main()
