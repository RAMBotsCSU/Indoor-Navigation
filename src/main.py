import sys
import asyncio
import os
from PyQt6.QtWidgets import QApplication

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap
from RobotMonitor import RobotMonitor  # Your UI class showing map + robot pose


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


async def autosave_heatmap(running_map, interval_s=5, out_dir="outputs"):
    """
    Periodically save accumulated heatmap every interval_s seconds.
    """
    os.makedirs(out_dir, exist_ok=True)
    step = 0
    while True:
        step += 1
        path = os.path.join(out_dir, f"map_step_{step:04d}.png")
        running_map.save_heatmap(path)
        print(f"[Heatmap] saved {path}")
        await asyncio.sleep(interval_s)


async def async_setup():
    """Initialize controller, odometry, LiDAR, and map."""
    # --- Motion Controller ---
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    # --- Odometry Estimator ---
    odom_estimator = OdometryEstimator()
    await odom_estimator.connect()
    asyncio.create_task(odom_estimator.start(rate_hz=200))  # continuous updates

    # --- LiDAR ---
    lidar = LiDAR('/dev/ttyUSB0')
    await lidar.connect()
    await asyncio.sleep(1.0)  # Give LiDAR time to stabilize
    lidar.start(asyncio.get_running_loop())

    # --- Running Map ---
    running_map = RunningMap(
        grid_size=200,
        cell_size_cm=5,
        max_distance_mm=6000
    )

    # --- Fusion loop ---
    asyncio.create_task(fusion_loop(lidar, odom_estimator, running_map))

    # --- Heatmap autosave ---
    asyncio.create_task(autosave_heatmap(running_map, interval_s=5, out_dir="outputs"))

    return controller, odom_estimator, lidar, running_map


def main():
    """Qt main thread; asyncio in background."""
    app = QApplication(sys.argv)
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    # --- Async setup ---
    controller, odom_estimator, lidar, running_map = loop.run_until_complete(async_setup())

    # --- UI ---
    ui = RobotMonitor(odom_estimator, running_map)
    ui.show()

    # --- Motion script ---
    motion_task = asyncio.create_task(motion_script(controller))

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

        # --- Save final map ---
        final_path = os.path.join("outputs", "map_final.png")
        running_map.save_heatmap(final_path)
        print(f"[Heatmap] Final map saved: {final_path}")

        loop.stop()
        loop.close()


if __name__ == "__main__":
    main()
