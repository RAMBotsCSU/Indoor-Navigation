import numpy as np

def analyze_csv(path):
    # Load CSV using numpy
    data = np.loadtxt(path, delimiter=",")
    
    print("=== CSV Analysis ===")
    print(f"Shape: {data.shape}")          # rows, columns
    print(f"Min: {np.min(data)}")
    print(f"Max: {np.max(data)}")
    print(f"Mean: {np.mean(data)}")
    print(f"Median: {np.median(data)}")
    print(f"Standard Deviation: {np.std(data)}")

if __name__ == "__main__":
    analyze_csv("include/lidar_data_old.csv")  # change this to your file
