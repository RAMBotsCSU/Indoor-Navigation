import asyncio
import csv

def read_csv(file_path):
    data = []
    with open(file_path, newline='') as csvfile:
        reader = csv.reader(csvfile)
        for row in reader:
            data.append(row)
    return data

async def example_move():
    data = read_csv('lidar_data_old.csv')
    for line in data:
        await movement(line)

async def movement(line):
    closest = 1e6
    for item in line:
        if float(item) > 1.0:
            closest = min(float(item), closest)
    
    print("Closest distance:", closest)
    if closest > 30:
        print("Forward")
        await asyncio.sleep(1)
    elif closest <= 30:
        print("Turn")
        await asyncio.sleep(1)

async def main():
    task = asyncio.create_task(example_move())
    await task     # or: await asyncio.Event().wait() if you want manual control

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Exiting")
