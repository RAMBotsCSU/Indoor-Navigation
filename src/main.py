import asyncio
import signal
import os
import time
from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap

OUTPUT_DIR = "outputs"
CAPTURE_INTERVAL = 0.5  # seconds

async def fusion_loop(lidar, odom, running_map):
    """Continuously integrate LiDAR points into the map."""
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)

async def capture_heatmap_loop(running_map, interval=CAPTURE_INTERVAL):
    """Optional: save intermediate heatmaps during mapping."""
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    step = 0
    while True:
        step += 1
        path = os.path.join(OUTPUT_DIR, f"map_step_{step:04d}.png")
        running_map.save_heatmap(path)
        print(f"[Heatmap] saved {path}")
        await asyncio.sleep(interval)

async def main():
    # --- Initialize odometry ---
    odom = OdometryEstimator()
    await odom.connect()
    asyncio.create_task(odom.start(rate_hz=50))

    # --- Initialize robot controller ---
    controller = RobotController()
    await controller.connect()
    await controller.enable()

    # --- Initialize LiDAR ---
    lidar = LiDAR('/dev/ttyUSB0')
    await lidar.connect()
    lidar.start(asyncio.get_running_loop())

    # --- Initialize running map ---
    running_map = RunningMap(grid_size=400, cell_size_cm=5, max_distance_mm=4000)

    # --- Start fusion loop ---
    asyncio.create_task(fusion_loop(lidar, odom, running_map))
    asyncio.create_task(capture_heatmap_loop(running_map))  # optional intermediate saves

    try:
        print(f"Trial 3: Two Turns Around Two Corners")
        await controller.forward_cm(305)
        await asyncio.sleep(0.05)
        await controller.turn_deg(90)
        await asyncio.sleep(0.05)
        await controller.forward_cm(1820)
        await asyncio.sleep(0.05)
        await controller.turn_deg(90)
        await asyncio.sleep(0.05)
        await controller.forward_cm(305)

    except KeyboardInterrupt:
        print("User interrupted")

    finally:
        # --- Cleanup ---
        print("Stopping...")
        await lidar.stop()
        await controller.stop()
        odom.stop()

        # --- Save final clean heatmap ---
        final_path = os.path.join(OUTPUT_DIR, "map_final.png")
        running_map.save_heatmap(final_path)
        print(f"Final heatmap saved: {final_path}")
        
        # --- Save binary occupancy map ---
        binary_path = os.path.join(OUTPUT_DIR, "map_binary.png")
        running_map.save_binary_map(binary_path, threshold=0.6)
        print(f"Binary map saved: {binary_path}")
        
        # --- Save overview map ---
        overview_path = os.path.join(OUTPUT_DIR, "map_overview.png")
        running_map.save_overview_map(overview_path)
        print(f"Overview map saved: {overview_path}")

if __name__ == "__main__":
    asyncio.run(main())
