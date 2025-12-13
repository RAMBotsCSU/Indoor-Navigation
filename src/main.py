from LiDAR import LiDAR
from Odometry import Odometry
from RunningMap import RunningMap
from LiDAR_UI import LiDAR_UI
import asyncio
import os


async def main():
    lidar = None
    odo = None
    ui = None
    
    try:
        # initialize components
        lidar = LiDAR('/dev/ttyUSB0')
        await lidar.connect()
        rm = RunningMap(grid_size=200, cell_size_cm=5, max_distance_mm=6000)
        odo = Odometry()
        
        # connect and enable odometry
        await odo.connect()
        await odo.enable()
        print("Odometry enabled")

        # initialize UI
        ui = LiDAR_UI(lidar, rm, odo)
        
        # start UI in background task
        ui_task = asyncio.create_task(ui.run())

        # prepare outputs directory (absolute path)
        project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        out_dir = os.path.join(project_root, "outputs")
        os.makedirs(out_dir, exist_ok=True)

        # take 100 pictures: every 10 cm forward (total 10 meters)
        for i in range(100):
            print(f"Moving forward 10 cm (step {i+1}/100)")
            success = await odo.forward_cm(10.0)
            print(f"Movement {'succeeded' if success else 'timed out'}")

            # scan LiDAR and get pose
            scan = await lidar.get_scan()
            x, y, theta = odo.pose()
            print(f"Robot pose: x={x:.2f} cm, y={y:.2f} cm, theta={theta:.4f} rad")

            # update map (UI will automatically update)
            rm.heatmap_from_scan(scan, position=(x, y))
            
            # save snapshot
            step_path = os.path.join(out_dir, f"map_step_{i+1:03d}.png")
            rm.save_overall_map(step_path)
            print(f"Saved step image to {os.path.abspath(step_path)}")
            
            # small delay to allow UI to update
            await asyncio.sleep(0.1)

        # save final accumulated map
        final_path = os.path.join(out_dir, "map_final.png")
        rm.save_overall_map(final_path)
        print(f"Saved final accumulated map to {os.path.abspath(final_path)}")

        # wait a bit for final UI update
        await asyncio.sleep(1.0)
        
        # stop UI
        if ui:
            await ui.stop()
        await ui_task

    except KeyboardInterrupt:
        print("Interrupted by user")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # cleanup
        print("Cleaning up...")
        if ui:
            try:
                await ui.stop()
            except Exception as e:
                print(f"Error stopping UI: {e}")
        if lidar:
            try:
                lidar.stop()
            except Exception as e:
                print(f"Error stopping LiDAR: {e}")
        if odo:
            try:
                odo.stop()
            except Exception as e:
                print(f"Error stopping Odometry: {e}")

if __name__ == "__main__":
    asyncio.run(main())