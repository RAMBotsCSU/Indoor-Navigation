# quick_plot_lidar.py
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# adjust path, delimiter, and column names to match your CSV
df = pd.read_csv(r'lidar_data_old.csv', sep=',')
# common column name variants: ['x','y','z'] or ['range','angle'] or ['r','theta']
if {'range','angle'}.issubset(df.columns):
    r = df['range'].astype(float)
    theta = np.deg2rad(df['angle'].astype(float))  # use deg2rad if angles in degrees
    x = r * np.cos(theta); y = r * np.sin(theta); z = df.get('z', pd.Series(0, index=df.index)).astype(float)
else:
    x = df['x'].astype(float); y = df['y'].astype(float); z = df.get('z', pd.Series(0, index=df.index)).astype(float)

mask = np.isfinite(x) & np.isfinite(y)
plt.figure(figsize=(6,6))
plt.scatter(x[mask], y[mask], c=z[mask], s=1, cmap='viridis')
plt.xlabel('X'); plt.ylabel('Y'); plt.axis('equal'); plt.colorbar(label='Z/intensity')
plt.show()