"""The fixed canyon-transit mission, flown identically by both trials.

Both trials must fly the SAME reference or the comparison is meaningless,
so this is a pure function of mission time with no feedback and no
randomness whatsoever.
"""
import numpy as np

from . import canyon_geometry as cg
from . import frames

CRUISE_SPEED = 12.0  # m/s along the canyon; inside the tiltrotor's transition band

WAYPOINTS_NED = [
    frames.enu_to_ned(cg.CANYON_ENTRY),
    frames.enu_to_ned(cg.CANYON_EXIT),
]


class Mission:
    """Straight transit from canyon entry to exit at constant speed."""

    def __init__(self, hold_s=3.0, speed=CRUISE_SPEED):
        self.hold_s = float(hold_s)
        self.start = WAYPOINTS_NED[0].astype(float)
        self.end = WAYPOINTS_NED[1].astype(float)
        delta = self.end - self.start
        self.distance = float(np.linalg.norm(delta))
        self.direction = delta / self.distance
        self.transit_s = self.distance / float(speed)
        self.speed = float(speed)

    def target(self, t):
        """NED position setpoint at mission time t, and a done flag."""
        t = float(t)
        if t <= self.hold_s:
            return self.start.copy(), False
        travelled = (t - self.hold_s) * self.speed
        if travelled >= self.distance:
            return self.end.copy(), True
        return self.start + self.direction * travelled, False
