"""Mission uploader. Baseline mode uploads a fixed AUTO mission and lets
ArduPilot's own navigation controller fly it; treatment mode additionally
nudges the cruise waypoint with a CBF-filtered PINN wind correction.

Earlier this session this node instead ran a hand-rolled RC-override
position/altitude/heading P/D loop (ArduPilot exposes no live GUIDED-mode
XY position/velocity injection for this airframe, confirmed by reading
ArduPlane/GCS_MAVLink_Plane.cpp -- handle_set_position_target_local_ned
is GUIDED-only and does altitude only). That loop was replaced after a
live demo watched it drift laterally under wind and strike a canyon
tower: AUTO-mode MISSION navigation is a different, unblocked ArduPilot
code path (a pre-uploaded MAVLink mission, not live position-target
injection) -- see docs/superpowers/specs/
2026-09-02-auto-mission-navigation-design.md for the full account,
including the QuadPlane::in_vtol_auto() source evidence that a
[NAV_VTOL_TAKEOFF, NAV_WAYPOINT, NAV_VTOL_LAND] mission stays in
Q-mode/VTOL navigation the whole way, never transitioning to fixed-wing.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, Waypoint
from mavros_msgs.srv import CommandBool, CommandLong, WaypointPush
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from . import canyon_geometry as cg
from . import constants as C
from . import frames
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .rc_pwm import arm, set_mode

SETPOINTS_BEFORE_OFFBOARD = 20  # give MAVROS/ArduPilot a beat to settle before requesting anything
ENGAGE_RETRY_TICKS = 50  # retry the mission-upload/arm+AUTO request once a second until it sticks
MODE_AUTO = 10  # ArduPlane custom mode number, confirmed against ArduPlane/mode.h

# Matches the README's --home value -- the ArduPilot EKF origin/home
# point this project's SITL runs boot with. Mission waypoints are
# uploaded as lat/lon relative to THIS point (frames.ned_to_latlon), not
# the vehicle's own local frame -- MAVROS's WaypointPush takes global
# coordinates, not local NED offsets.
HOME_LAT, HOME_LON = 44.434424990487216, 26.04781615647584
# 25m relative-to-home (Waypoint.z_alt under FRAME_GLOBAL_REL_ALT), not
# an absolute value -- unaffected by canyon_geometry.GROUND_Z, since
# ArduPilot's home-relative altitude is a delta from wherever the
# vehicle armed, and the vehicle's own spawn height moves by the same
# offset as GROUND_Z (see run_trial.SPAWN_XYZ). Chosen to match
# canyon_geometry.CANYON_ENTRY's height above the ground it spawns on.
CRUISE_ALT_M = 25.0

# How far past the canyon exit the mission's cruise/landing point sits,
# measured from the far edge of the LAST tower row (tower_2_n/tower_2_s)
# -- not canyon_geometry.CANYON_EXIT, which is set generously past the
# towers (45m of margin) for the OLD hand-rolled trajectory-follower's
# own stability needs. That reasoning doesn't apply to ArduPilot's own
# navigation controller, so the mission targets the real landing point
# directly -- see the design spec for the full account. Consumed by
# _build_mission() below, not a per-tick position check any more (the
# old position-triggered landing logic is gone; ArduPilot's own mission
# sequencer owns arrival/landing/disarm entirely once the mission
# reaches its NAV_VTOL_LAND item).
LAND_CLEARANCE_M = 2.0
_LAST_TOWER_EDGE_ENU_X = max(b.cx + b.sx / 2.0 for b in cg.BUILDINGS if b.cx > 0)
LAND_TRIGGER_LOCAL_M = _LAST_TOWER_EDGE_ENU_X + LAND_CLEARANCE_M

ENABLE_VTOL_TRANSITION = False  # unchanged -- see cbf_filter.py's own use of this flag


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('mode', 'baseline')
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('baseline', 'treatment'):
            raise ValueError(f'mode must be baseline or treatment, got {self.mode}')
        self.get_logger().info(f'controller mode: {self.mode}')

        # Scales the PINN feedforward before it reaches the CBF. See the
        # mission-stack port spec for the full measured-live rationale
        # behind 0.2 (unchanged from the RC-override design -- this only
        # affects u_des, not how the correction reaches the vehicle).
        self.declare_parameter('feedforward_gain', 0.2)
        self.ff_gain = float(self.get_parameter('feedforward_gain').value)

        self.tick = 0
        self.mission_uploaded = False
        self.wind_est = np.zeros(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.wind_truth = np.zeros(3)
        self._waypoint_offset = np.zeros(2)
        self._last_offset_push_tick = 0

        # The CBF's stall barrier is a fixed-wing (loss-of-lift) concept
        # -- see cbf_filter.py's module docstring.
        self.cbf = CBFFilter(enable_stall=ENABLE_VTOL_TRANSITION)
        self.cbf_pub = self.create_publisher(
            Vector3Stamped, C.TOPIC_CBF_DIAG, 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.mission_client = self.create_client(WaypointPush, '/mavros/mission/push')

        self.mavros_connected = False
        self.mavros_armed = False

        self.create_subscription(
            State, '/mavros/state', self._on_state, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_EST, self._on_wind_est, 10)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_TRUTH, self._on_wind_truth, 10)

        self.create_timer(1.0 / C.CONTROL_HZ, self._tick)

    def _on_state(self, msg):
        self.mavros_connected = msg.connected
        self.mavros_armed = msg.armed

    def _on_pose(self, msg):
        p = msg.pose.position
        self.pos = frames.enu_to_ned([p.x, p.y, p.z])

    def _on_velocity(self, msg):
        v = msg.twist.linear
        self.vel = frames.enu_to_ned([v.x, v.y, v.z])

    def _on_imu(self, msg):
        q = msg.orientation
        self.quat = frames.enu_flu_quat_to_ned_frd([q.w, q.x, q.y, q.z])

    def _on_wind_est(self, msg):
        self.wind_est = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _on_wind_truth(self, msg):
        self.wind_truth = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _build_mission(self):
        """[home placeholder, NAV_VTOL_TAKEOFF @ entry, NAV_WAYPOINT @
        landing-trigger point, NAV_VTOL_LAND @ landing-trigger point] --
        all at CRUISE_ALT_M, all in Q-mode/VTOL navigation the whole way
        (QuadPlane::in_vtol_auto() latches true from the takeoff item and
        never auto-clears without an explicit transition command, which
        this project never issues). Landing targets the REAL landing-
        trigger point (last tower row's edge + LAND_CLEARANCE_M), not
        CANYON_EXIT -- see the design spec for why the old 45m margin
        doesn't apply to ArduPilot's own navigation controller.

        Item 0 is a placeholder: ArduPilot always treats mission seq 0
        as the home position and overwrites/ignores whatever is uploaded
        there (confirmed live -- pushing [TAKEOFF, WAYPOINT, LAND]
        starting at seq 0 came back from /mavros/mission/waypoints with
        item 0 silently replaced by an all-zero home entry and the real
        TAKEOFF item dropped). The real mission starts at seq 1."""
        entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
        land_ned = np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M, entry_ned[2]])

        def wp(command, ned, is_current=False):
            lat, lon = frames.ned_to_latlon(ned, HOME_LAT, HOME_LON)
            w = Waypoint()
            w.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            w.command = command
            w.is_current = is_current
            w.autocontinue = True
            w.x_lat = lat
            w.y_long = lon
            w.z_alt = CRUISE_ALT_M
            return w

        return [
            wp(16, entry_ned),                    # seq 0: home placeholder, overwritten by ArduPilot
            wp(84, entry_ned, is_current=True),   # MAV_CMD_NAV_VTOL_TAKEOFF
            wp(16, land_ned),                     # MAV_CMD_NAV_WAYPOINT
            wp(85, land_ned),                     # MAV_CMD_NAV_VTOL_LAND
        ]

    @staticmethod
    def _accumulate_offset(u_safe, dt, current_offset, max_offset_m):
        """Kinematic displacement over one update interval (0.5*a*dt^2),
        added to the running offset and clamped to max_offset_m -- so a
        runaway correction can't push the mission waypoint somewhere
        unsafe. Horizontal (NED north/east) only; altitude stays flown
        by the mission's own fixed CRUISE_ALT_M."""
        delta = 0.5 * np.asarray(u_safe[:2], dtype=float) * dt * dt
        new_offset = np.asarray(current_offset, dtype=float) + delta
        mag = np.linalg.norm(new_offset)
        if mag > max_offset_m:
            new_offset = new_offset * (max_offset_m / mag)
        return new_offset

    def _treatment_tick(self):
        u_des = -self.ff_gain * self.wind_est / MASS_KG
        u_safe, info = self.cbf.filter(u_des, self.pos, self.vel,
                                       self.wind_truth, self.quat)

        diag = Vector3Stamped()
        diag.header.stamp = self.get_clock().now().to_msg()
        diag.vector.x = 1.0 if info['active'] else 0.0
        # Obstacle barrier only (metres) -- see cbf_filter.py for why
        # this must never be combined with the stall barrier (radians).
        diag.vector.y = float(np.clip(info['h_obstacle'], -1e3, 1e3))
        diag.vector.z = 0.0 if info['feasible'] else 1.0
        self.cbf_pub.publish(diag)

        self._waypoint_offset = self._accumulate_offset(
            u_safe, 1.0 / C.CONTROL_HZ, self._waypoint_offset,
            C.MAX_WAYPOINT_OFFSET_M)

        since_last_push = self.tick - self._last_offset_push_tick
        if since_last_push >= C.CONTROL_HZ / C.CORRECTION_UPDATE_HZ:
            entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
            land_ned = np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M, entry_ned[2]])
            corrected_ned = land_ned + np.array(
                [self._waypoint_offset[0], self._waypoint_offset[1], 0.0])
            lat, lon = frames.ned_to_latlon(corrected_ned, HOME_LAT, HOME_LON)

            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = CRUISE_ALT_M

            req = WaypointPush.Request()
            req.start_index = 2  # seq 2 -- the cruise NAV_WAYPOINT item (see _build_mission's seq-0-is-home note)
            req.waypoints = [wp]
            self.mission_client.call_async(req)
            self._last_offset_push_tick = self.tick

    def _tick(self):
        if not self.mission_uploaded:
            since_start = self.tick - SETPOINTS_BEFORE_OFFBOARD
            if since_start >= 0 and since_start % ENGAGE_RETRY_TICKS == 0:
                req = WaypointPush.Request()
                req.start_index = 0
                req.waypoints = self._build_mission()
                self.mission_client.call_async(req)
                self.get_logger().info('requested mission upload')
                # Optimistic -- the arm/engage retry below already
                # tolerates a request landing on a not-yet-ready FCU by
                # simply asking again, so a real ack-based state machine
                # isn't needed here.
                self.mission_uploaded = True

        engaged = self.mavros_armed
        since_stream_started = self.tick - SETPOINTS_BEFORE_OFFBOARD - ENGAGE_RETRY_TICKS
        if (not engaged and since_stream_started >= 0
                and since_stream_started % ENGAGE_RETRY_TICKS == 0):
            set_mode(self.cmd_client, MODE_AUTO)
            arm(self.arm_client, True)
            self.get_logger().info('requested AUTO mode and arm')

        if self.mode == 'treatment':
            self._treatment_tick()

        self.tick += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
