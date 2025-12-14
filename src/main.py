import asyncio
import signal
import os
import time

from LiDAR import LiDAR
from RobotController import RobotController
from OdometryEstimator import OdometryEstimator
from RunningMap import RunningMap


# =========================
# LiDAR → Odometry → Map
# =========================
async def fusion_loop(lidar, odom, running_map):
    while True:
        ts, angle, dist = await lidar.queue.get()
        pose = odom.interpolate(ts)
        running_map.integrate_point(angle, dist, pose)


# =========================
# Example robot motion
# =========================
async def motion_script(controller):
    try:
        while True:
            print("Forward 10 cm")
            await controller.forward_cm(10)
            await asyncio.sleep(0.2)

            print("Turn 15 deg")
            await controller.turn_deg(15)
            await asyncio.sleep(0.2)
    except asyncio.CancelledError:
        print("[Motion] stopped")


# =========================
# Periodic map capture
# =========================
async def save_map_frames(running_map, interval=1.0, out_dir="outputs"):
    os.makedirs(out_dir, exist_ok=True)
    frame = 0

    while True:
        await asyncio.sleep(interval)
        frame += 1

        path = os.path.join(out_dir, f"map_{frame:04d}.png")
        running_map.save_heatmap(path)

        print(f"[Map] saved {path}")


# =========================
# Setup all subsystems
# =========================
async def setup():
    # --- Motion controller ---
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
    await asyncio.sleep(1.0)
    lidar.start(asyncio.get_running_loop())

    # --- Map ---
    running_map = RunningMap(
        grid_size=200,
        cell_size_cm=5,
        max_distance_mm=6000
    )

    return controller, odom, lidar, running_map


# =========================
# Main entry
# =========================
def main():
    # Allow Ctrl+C to work properly
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)

    controller = odom = lidar = running_map = None
    tasks = []

    try:
        controller, odom, lidar, running_map = loop.run_until_complete(setup())

        tasks.extend([
            loop.create_task(fusion_loop(lidar, odom, running_map)),
            loop.create_task(motion_script(controller)),
            loop.create_task(save_map_frames(running_map, interval=1.0))
        ])

        print("[System] running — Ctrl+C to stop")
        loop.run_forever()

    except KeyboardInterrupt:
        print("\n[System] shutdown requested")

    finally:
        print("[System] cleaning up...")

        for t in tasks:
            t.cancel()

        loop.run_until_complete(
            asyncio.gather(*tasks, return_exceptions=True)
        )

        if lidar:
            loop.run_until_complete(lidar.stop())
        if controller:
            loop.run_until_complete(controller.stop())
        if odom:
            odom.stop()

        # Save final map
        os.makedirs("outputs", exist_ok=True)
        final_path = "outputs/map_final.png"
        running_map.save_heatmap(final_path)
        print(f"[Map] final saved → {final_path}")

        loop.stop()
        loop.close()
        print("[System] exited cleanly")


if __name__ == "__main__":
    main()
