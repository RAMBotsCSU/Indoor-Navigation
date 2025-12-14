import time

class PoseProvider:
    """
    Pose provider with mode switching and optional offset alignment.

    Traditional: returns odom pose (cm, cm, rad)

    RSSI_ML: uses ML to define a GLOBAL map frame:
      - On first confident ML reading, locks an (offset_x, offset_y)
        so odometry is aligned to the ML map frame.
      - Then returns (odom + offset) as the default global pose.
      - If ML confidence is high: snap or blend the global position and update offset.
    """

    def __init__(self, odom_estimator, map_units_to_cm: float):
        self.odom = odom_estimator
        self.map_units_to_cm = float(map_units_to_cm)

        self.mode = "traditional"
        self.last_ml = None  # (x_cm, y_cm, conf, t)

        # gating / fusion params
        self.conf_accept = 0.80
        self.conf_soft = 0.65
        self.alpha = 0.15

        # offset alignment
        self.offset_locked = False
        self.offset_x = 0.0
        self.offset_y = 0.0

    def set_mode(self, mode: str):
        self.mode = mode

    def update_ml_map_units(self, x_map: float, y_map: float, conf: float):
        x_cm = float(x_map) * self.map_units_to_cm
        y_cm = float(y_map) * self.map_units_to_cm
        self.last_ml = (x_cm, y_cm, float(conf), time.time())

    def _ensure_offset(self, odom_x, odom_y):
        """
        Lock offset when we first get a confident ML estimate.
        """
        if self.offset_locked:
            return

        if self.last_ml is None:
            return

        mx, my, conf, _t = self.last_ml
        if conf >= self.conf_accept:
            self.offset_x = mx - odom_x
            self.offset_y = my - odom_y
            self.offset_locked = True

    def _global_from_odom(self, odom_x, odom_y):
        return odom_x + self.offset_x, odom_y + self.offset_y

    def _update_offset_from_global_target(self, odom_x, odom_y, gx, gy):
        """
        If we decide the global pose should be (gx, gy),
        update offset so (odom + offset) matches that.
        """
        self.offset_x = gx - odom_x
        self.offset_y = gy - odom_y
        self.offset_locked = True

    def _fuse(self, ox: float, oy: float, oth: float):
        """
        Returns fused pose (global_x_cm, global_y_cm, oth_rad).
        """
        # Traditional: raw odom
        if self.mode != "rssi_ml":
            return ox, oy, oth

        # In ML mode, first align odom to ML frame when we can
        self._ensure_offset(ox, oy)

        # Default global pose comes from odom + offset (if locked)
        if self.offset_locked:
            gx, gy = self._global_from_odom(ox, oy)
        else:
            # if not locked yet, just return odom until ML becomes confident
            gx, gy = ox, oy

        if self.last_ml is None:
            return gx, gy, oth

        mx, my, conf, _t = self.last_ml

        # High confidence -> snap global to ML and update offset accordingly
        if conf >= self.conf_accept:
            self._update_offset_from_global_target(ox, oy, mx, my)
            return mx, my, oth

        # Medium confidence -> blend global toward ML (and update offset to match blended global)
        if conf >= self.conf_soft and self.offset_locked:
            a = self.alpha
            bx = (1 - a) * gx + a * mx
            by = (1 - a) * gy + a * my
            self._update_offset_from_global_target(ox, oy, bx, by)
            return bx, by, oth

        # Low confidence -> ignore ML, keep global from odom+offset (or odom if not locked)
        return gx, gy, oth

    def pose(self):
        ox, oy, oth = self.odom.pose()
        return self._fuse(ox, oy, oth)

    def interpolate(self, ts: float):
        ox, oy, oth = self.odom.interpolate(ts)
        return self._fuse(ox, oy, oth)