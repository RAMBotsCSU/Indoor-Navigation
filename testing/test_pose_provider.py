import time
from PoseProvider import PoseProvider

class FakeOdom:
    def __init__(self):
        self.x, self.y, self.th = 10.0, 20.0, 0.5  # cm, cm, rad

    def pose(self):
        return self.x, self.y, self.th

    def interpolate(self, ts):
        return self.pose()

odom = FakeOdom()
pp = PoseProvider(odom, map_units_to_cm=100/14.2)

print("TRAD pose:", pp.pose())

pp.set_mode("rssi_ml")
pp.update_ml_map_units(545, 260, 0.95)  # map-units
print("ML pose (snap):", pp.pose())

pp.update_ml_map_units(545, 260, 0.70)  # soft accept
print("ML pose (blend):", pp.pose())

pp.update_ml_map_units(545, 260, 0.20)  # reject
print("ML pose (reject -> odom):", pp.pose())