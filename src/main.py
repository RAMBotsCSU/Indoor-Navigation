from LiDAR import LiDAR
from Odometry import Odometry
from RunningMap import RunningMap
import asyncio


async def main():
    try:
        # initialize components
        lidar = LiDAR('/dev/ttyUSB0')
        rm = RunningMap(grid_size=200, cell_size_cm=5, max_distance_mm=6000)
        odo = Odometry()
        
        # connect and enable odometry
        await odo.connect()
        await odo.enable()
        print("Odometry enabled")

        # move forward 1m
        print("Moving forward 1 m")
        success = await odo.forward_cm(100.0)
        print(f"Movement {'succeeded' if success else 'timed out'}")

        # scan LiDAR and get pose
        scan = lidar.get_scan()
        x, y, theta = odo.pose()
        print(f"Robot pose: x={x:.2f}, y={y:.2f}, theta={theta:.4f}")

        # update map
        frame = rm.heatmap_from_scan(scan, position=(x, y))
        print(f"Heatmap updated, accumulated {rm.num_frames()} frame(s)")

        # save map
        rm.save_overall_map("outputs/map.png")
        print("Map saved to outputs/map.png")

        lidar.stop()

    except KeyboardInterrupt:
        print("Interrupted by user")
        try:
            lidar.stop()
            odo.stop()
        except Exception:
            pass
    except Exception as e:
        print(f"Error: {e}")
        try:
            lidar.stop()
            odo.stop()
        except Exception:
            pass

if __name__ == "__main__":
    asyncio.run(main())