import numpy as np
import pandas as pd

class RunningMap:
    def __init__(self, csv_path, time_col="time", x_col="x", y_col="y", v_col="value"):
        self.csv_path = csv_path
        self.time_col = time_col
        self.x_col = x_col
        self.y_col = y_col
        self.v_col = v_col
        self.times = None
        self.heat = None  # shape: (T, H, W)
        self._load_and_build()

    def _load_and_build(self):
        df = pd.read_csv(self.csv_path)
        # ensure required columns
        for c in [self.time_col, self.x_col, self.y_col, self.v_col]:
            if c not in df.columns:
                raise ValueError(f"Missing column: {c}")

        # sort by time for consistency
        df = df.sort_values(by=self.time_col)

        # unique sorted times
        self.times = np.sort(df[self.time_col].unique())

        # infer grid size from max x/y (assuming 0-based integer grid)
        max_x = int(df[self.x_col].max())
        max_y = int(df[self.y_col].max())
        W = max_x + 1
        H = max_y + 1

        # map time -> index
        t_to_idx = {t: i for i, t in enumerate(self.times)}

        # allocate tensor
        self.heat = np.zeros((len(self.times), H, W), dtype=float)

        # aggregate values (if duplicates, accumulate)
        for _, row in df.iterrows():
            ti = t_to_idx[row[self.time_col]]
            xi = int(row[self.x_col])
            yi = int(row[self.y_col])
            self.heat[ti, yi, xi] += float(row[self.v_col])

    def num_frames(self):
        return len(self.times)

    def frame_at_time(self, t, nearest=True):
        """Return heatmap for exact time t; if nearest=True, pick nearest time."""
        if nearest:
            idx = int(np.argmin(np.abs(self.times - t)))
        else:
            matches = np.where(self.times == t)[0]
            if len(matches) == 0:
                raise KeyError(f"time {t} not found")
            idx = int(matches[0])
        return self.heat[idx]

    def frame_at_index(self, idx):
        """Return heatmap at frame index (0-based)."""
        return self.heat[idx]

    def show_frame(self, idx=0, cmap="hot"):
        import matplotlib.pyplot as plt
        plt.imshow(self.frame_at_index(idx), cmap=cmap, origin="lower")
        plt.title(f"t={self.times[idx]}")
        plt.colorbar(label=self.v_col)
        plt.tight_layout()
        plt.show()

rm = RunningMap("include/lidar_data_old.csv", time_col="t", x_col="i", y_col="j", v_col="val")
rm.show_frame(0)
frame = rm.frame_at_time(5.0)
