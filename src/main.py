from LiDAR import LiDAR
from Odometry import Odometry
from RunningMap import RunningMap
import asyncio
import os
import signal
import sys
import subprocess
import time

# Global variables for cleanup
lidar = None
odo = None
shutdown_event = asyncio.Event()


def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully."""
    print("\nShutdown signal received, cleaning up...")
    shutdown_event.set()

async def cleanup():
    """Clean up resources."""
    global lidar, odo
    try:
        if lidar:
            print("Stopping LiDAR...")
            lidar.stop()
        if odo:
            print("Stopping Odometry...")
            odo.stop()
    except Exception as e:
        print(f"Error during cleanup: {e}")


async def main():
    global lidar, odo
    try:        
        # initialize components
        lidar = LiDAR('/dev/ttyUSB0')
        rm = RunningMap(grid_size=200, cell_size_cm=5, max_distance_mm=6000)
        odo = Odometry()
        
        # connect and enable odometry
        await odo.connect()
        await odo.enable()
        print("Odometry enabled")

        # prepare outputs directory (absolute path)
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        out_dir = os.path.join(project_root, "outputs")
        os.makedirs(out_dir, exist_ok=True)

        # take 10 pictures: every 1 meter forward
        for i in range(10):
            if shutdown_event.is_set():
                print("Shutdown requested, stopping...")
                break
            
            print(f"Moving forward 1 m (step {i+1}/10)")
            success = await odo.forward_cm(100.0)
            print(f"Movement {'succeeded' if success else 'timed out'}")

            # scan LiDAR and get pose
            scan = lidar.get_scan()
            x, y, theta = odo.pose()
            print(f"Robot pose: x={x:.2f} cm, y={y:.2f} cm, theta={theta:.4f} rad")

            # update map and save snapshot
            rm.heatmap_from_scan(scan, position=(x, y))
            step_path = os.path.join(out_dir, f"map_step_{i+1:02d}.png")
            rm.save_overall_map(step_path)
            print(f"Saved step image to {os.path.abspath(step_path)}")

        # optional: save final accumulated map as a separate file
        final_path = os.path.join(out_dir, "map_final.png")
        rm.save_overall_map(final_path)
        print(f"Saved final accumulated map to {os.path.abspath(final_path)}")

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        await cleanup()
        sys.exit(0)

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    asyncio.run(main())