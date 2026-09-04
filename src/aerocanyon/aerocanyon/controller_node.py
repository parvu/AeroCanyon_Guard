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
import json
import pathlib

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State, Waypoint, WaypointList
from mavros_msgs.srv import (CommandBool, CommandLong, WaypointPush,
                             WaypointSetCurrent)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from . import canyon_geometry as cg
from . import constants as C
from . import frames
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .rc_pwm import arm, pwm, set_mode

# Raised from 20 -- belt-and-suspenders alongside _mission_confirmed
# (below) for the mission-complete-instantly bug documented near
# ENABLE_VTOL_TRANSITION. _mission_confirmed fixes the CONFIRMED
# mechanism (AUTO engaging before the mission upload actually landed on
# the FCU). This wider pre-arm wait guards a second, less-confirmed
# suspect: a failed FIRST arm+AUTO attempt (generic pre-arm checks not
# yet ready) forcing a retry ~1s later, during which AUTO mode may have
# already taken effect from the failed attempt even though arming
# hadn't -- the same "gap between mode-AUTO and actually armed" shape,
# by a different route. A longer pre-arm wait makes the first attempt
# more likely to land on an already-ready FCU, reducing (not
# eliminating) how often that retry -- and the gap -- happens at all.
SETPOINTS_BEFORE_OFFBOARD = 100
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
LAND_CLEARANCE_M = 5.0
_LAST_TOWER_EDGE_ENU_X = max(b.cx + b.sx / 2.0 for b in cg.BUILDINGS if b.cx > 0)
LAND_TRIGGER_LOCAL_M = _LAST_TOWER_EDGE_ENU_X + LAND_CLEARANCE_M

# seq 2 in _build_mission()'s list -- the single NAV_WAYPOINT the whole
# canyon transit flies toward. Shared by _treatment_tick's correction
# push and its follow-up WaypointSetCurrent restart -- see that method's
# docstring for why the restart is required at all.
CRUISE_WP_SEQ = 2

ENABLE_VTOL_TRANSITION = False  # unchanged -- see cbf_filter.py's own use of this flag

# Four engage-sequence variants were tried live, and every one, at least
# once, reproduced the same bug: ArduPilot marking the whole mission
# "complete" almost instantly (confirmed via STATUSTEXT -- "Throttle
# armed" straight to "Mission complete, changing mode to RTL", no
# waypoint navigation in between), then RTL tries a fixed-wing transition
# this VTOL-only airframe can never finish: (1) arm into QHOVER, climb to
# a target altitude, then AUTO; (2) arm into QHOVER, flat hold for a
# fixed delay, then AUTO; (3) arm into QSTABILIZE, climb, then AUTO; (4)
# arm and set_mode(AUTO) requested together, in the same instant --
# worked twice, failed once when a retry was needed (see
# SETPOINTS_BEFORE_OFFBOARD above).
#
# The CONFIRMED root cause, found by reading _tick()'s own mission-upload
# code: it was optimistic -- mission_uploaded was set True the instant
# the async WaypointPush request was FIRED, not when the FCU actually
# confirmed receiving it. If AUTO engaged before the mission had actually
# landed, ArduPilot sees zero/partial items and instantly reports
# "complete" -- independent of which pre-arm hold mode was used, which is
# exactly the pattern observed across all four variants. _mission_confirmed
# (see _tick()) gates the arm+AUTO engage sequence below on the FCU's own
# WaypointPush response, not just the request having been sent.
#
# Real bug found live 2026-09-04: the QSTABILIZE-climb-then-AUTO variant
# (arm into QSTABILIZE, hold a fixed RC throttle stick until
# QSTAB_CLIMB_ALT_M, only then switch to AUTO) was dropped at the user's
# request after a bad start under this exact sequence. The mission itself
# already opens with a NAV_VTOL_TAKEOFF item (command 84, see
# map_zone_demo.json) that climbs to its own target altitude -- AUTO's
# own takeoff handling does that job natively, so the custom QSTABILIZE
# climb was redundant machinery duplicating (and, live-verified,
# sometimes fighting) it. Now: arm straight into AUTO.

# QuadPlane's own AUTO-mode navigation never turns the nose to face the
# next waypoint outside the final landing approach -- confirmed against
# ArduPlane/quadplane.cpp: no WP_YAW_BEHAVIOR equivalent exists for this
# airframe type (that parameter is ArduCopter/ArduSub-only). It DOES read
# RC yaw (rudder) input during in_vtol_auto() as long as STICK_MIXING !=
# NONE (the SITL default, unset in tricopter.parm) -- see
# QuadPlane::get_pilot_input_yaw_rate_cds(). So _yaw_to_target() below
# drives a yaw-only /mavros/rc/override stream, a P controller on heading
# error toward the mission's cruise/landing target, leaving
# roll/pitch/throttle at CHAN_NOCHANGE so AUTO's own position/altitude
# control is untouched.
YAW_RATE_MAX_DEG_S = 60.0  # conservative cap, well under ACRO_YAW_RATE's 90 deg/s default
YAW_KP = 2.0  # rad/s commanded per rad heading error -- saturates above ~30 deg error


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('mode', 'baseline')
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('baseline', 'treatment'):
            raise ValueError(f'mode must be baseline or treatment, got {self.mode}')
        self.get_logger().info(f'controller mode: {self.mode}')

        self.declare_parameter('world', 'urban_canyon')
        self.world = self.get_parameter('world').value
        self.declare_parameter('mission_file', '')
        self.mission_file = self.get_parameter('mission_file').value

        # Scales the PINN feedforward before it reaches the CBF. See the
        # mission-stack port spec for the full measured-live rationale
        # behind 0.2 (unchanged from the RC-override design -- this only
        # affects u_des, not how the correction reaches the vehicle).
        self.declare_parameter('feedforward_gain', 0.2)
        self.ff_gain = float(self.get_parameter('feedforward_gain').value)

        self.tick = 0
        self.mission_uploaded = False
        self._mission_push_future = None
        self._mission_confirmed = False
        self._correction_push_future = None
        self.wind_est = np.zeros(3)
        self.pos = np.zeros(3)
        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.wind_truth = np.zeros(3)
        self._waypoint_offset = np.zeros(2)
        self._last_offset_push_tick = 0
        self._mission_current_seq = CRUISE_WP_SEQ
        self._mission_waypoints = []
        self.fcu_mode = ''
        self._was_armed = False
        self._mission_complete = False
        self._engage_phase = 'preflight'  # preflight -> auto

        # The CBF's stall barrier is a fixed-wing (loss-of-lift) concept
        # -- see cbf_filter.py's module docstring.
        self.cbf = CBFFilter(enable_stall=ENABLE_VTOL_TRANSITION)
        self.cbf_pub = self.create_publisher(
            Vector3Stamped, C.TOPIC_CBF_DIAG, 10)

        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.mission_client = self.create_client(WaypointPush, '/mavros/mission/push')
        self.set_current_client = self.create_client(
            WaypointSetCurrent, '/mavros/mission/set_current')
        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)

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
        self.create_subscription(
            WaypointList, '/mavros/mission/waypoints', self._on_mission_waypoints, 10)

        self.create_timer(1.0 / C.CONTROL_HZ, self._tick)

    def _on_state(self, msg):
        self.mavros_connected = msg.connected
        self.mavros_armed = msg.armed
        self.fcu_mode = msg.mode

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
        if self.world == 'map_zone':
            return self._build_map_zone_mission()
        return self._build_urban_canyon_mission()

    def _build_urban_canyon_mission(self):
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

    def _build_map_zone_mission(self):
        """Replay a mission captured by dump_mission.py from a live
        Mission Planner session (see README/dump_mission.py) verbatim:
        [home placeholder, item 0 (is_current), item 1, ..., item N-1] --
        same seq-0-placeholder shape as _build_urban_canyon_mission, for
        the same reason (ArduPilot always overwrites it). Each item's
        own command/frame/altitude is replayed as captured, unlike
        urban_canyon's fixed CRUISE_ALT_M -- the mission was authored at
        whatever altitudes made sense for the real terrain."""
        items = json.loads(pathlib.Path(self.mission_file).read_text())
        if not items:
            raise ValueError(f'mission file {self.mission_file} has no waypoints')

        def wp(item, is_current=False):
            w = Waypoint()
            w.frame = item['frame']
            w.command = item['command']
            w.is_current = is_current
            w.autocontinue = item['autocontinue']
            w.x_lat = item['x_lat']
            w.y_long = item['y_long']
            w.z_alt = item['z_alt']
            return w

        home_placeholder = wp(items[0])  # content is irrelevant -- ArduPilot overwrites seq 0
        return [home_placeholder] + [
            wp(item, is_current=(i == 0)) for i, item in enumerate(items)
        ]

    def _on_mission_waypoints(self, msg):
        self._mission_current_seq = msg.current_seq
        self._mission_waypoints = msg.waypoints

    def _active_target(self):
        """(north, east) NED metres and altitude (relative-alt, metres)
        of the mission's currently active nav waypoint -- read from the
        FCU's own reported mission (/mavros/mission/waypoints) once
        available. Before the first WaypointList arrives, falls back to
        the fixed urban_canyon land-trigger point at CRUISE_ALT_M -- the
        same point/seq (CRUISE_WP_SEQ, this class's own default
        _mission_current_seq) _treatment_tick has always corrected, so
        every world behaves exactly as before until MAVROS actually
        reports a mission."""
        if (self._mission_waypoints
                and self._mission_current_seq < len(self._mission_waypoints)):
            wp = self._mission_waypoints[self._mission_current_seq]
            north, east = frames.latlon_to_ned(wp.x_lat, wp.y_long, HOME_LAT, HOME_LON)
            return np.array([north, east]), float(wp.z_alt)
        entry_ned = frames.enu_to_ned(cg.CANYON_ENTRY)
        return np.array([entry_ned[0], LAND_TRIGGER_LOCAL_M]), CRUISE_ALT_M

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
        # Confirm the PREVIOUS correction push, then restart the cruise
        # nav command on it. Pushing a WaypointPush to an already-active
        # mission item only rewrites its entry in ArduPilot's mission
        # STORAGE -- confirmed against AP_Mission::replace_cmd's own doc
        # comment ("replacing the current active command will have no
        # effect until the command is restarted"): ArduPlane caches the
        # active nav command's target in RAM (_nav_cmd, set once by
        # advance_current_nav_cmd() when the command starts) and never
        # re-reads storage while it's running. Every correction this loop
        # computed was therefore silently discarded in flight -- baseline
        # and treatment flew identically, confirmed live by a 49-point
        # wind sweep landing at ~0% mean effect (noise-sized variance,
        # not a real one). MISSION_SET_CURRENT on the SAME index
        # (mavros's /mavros/mission/set_current -> AP_Mission::
        # set_current_cmd -> advance_current_nav_cmd) is ArduPilot's own
        # supported mechanism for restarting the active nav command in
        # place, forcing it to re-read the just-written target from
        # storage. Gated on the push's own future the same way
        # _mission_confirmed gates arm/AUTO above -- firing SET_CURRENT
        # before the FCU has actually finished the WaypointPush handshake
        # would restart onto the STALE, not-yet-updated location.
        if self._correction_push_future is not None and self._correction_push_future.done():
            result = self._correction_push_future.result()
            self._correction_push_future = None
            if result is not None and result.success:
                set_current_req = WaypointSetCurrent.Request()
                set_current_req.wp_seq = self._mission_current_seq
                self.set_current_client.call_async(set_current_req)

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
        # Also gated on the initial mission having actually been
        # confirmed, and on the previous correction push having resolved
        # (either confirmed above, or failed). ArduPilot's mission-item
        # protocol is a single stateful request/ack handshake shared by
        # EVERY WaypointPush regardless of target index -- live-verified
        # this collides: this method runs from tick 0 regardless of mode
        # state, so its first correction push (tick 50) landed BEFORE
        # the initial 4-item mission upload even starts (tick
        # SETPOINTS_BEFORE_OFFBOARD=100), and its second (tick 100) fired
        # on the SAME tick as that upload -- two concurrent WaypointPush
        # transfers on one handshake, which ArduPilot's mission protocol
        # cannot do, producing "Mission upload timeout" on the FCU and
        # leaving the vehicle without a real mission at all.
        if (self._mission_confirmed
                and since_last_push >= C.CONTROL_HZ / C.CORRECTION_UPDATE_HZ
                and self._correction_push_future is None):
            target_ned, target_alt = self._active_target()
            corrected_ned = target_ned + self._waypoint_offset
            lat, lon = frames.ned_to_latlon(
                np.array([corrected_ned[0], corrected_ned[1], 0.0]), HOME_LAT, HOME_LON)

            wp = Waypoint()
            wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            wp.command = 16  # MAV_CMD_NAV_WAYPOINT
            wp.is_current = False
            wp.autocontinue = True
            wp.x_lat = lat
            wp.y_long = lon
            wp.z_alt = target_alt

            req = WaypointPush.Request()
            req.start_index = self._mission_current_seq
            req.waypoints = [wp]
            self._correction_push_future = self.mission_client.call_async(req)
            self._last_offset_push_tick = self.tick

    def _yaw_to_target(self):
        """Stream a yaw-only RC override toward the mission's active
        cruise/landing target -- see the YAW_RATE_MAX_DEG_S/YAW_KP
        comment above for why this is needed at all. Runs in
        CMODE(MODE_AUTO), including during the mission's own
        NAV_VTOL_TAKEOFF climb (not just level cruise): AUTO's
        in_vtol_auto() reads RC yaw as long as STICK_MIXING != NONE (the
        SITL default, unset in tricopter.parm -- see the module-level
        comment above), so this correction is live from the moment AUTO
        engages, same as it was during the since-removed QSTABILIZE climb
        phase. Without it: the vehicle can enter AUTO facing nearly the
        OPPOSITE direction from its target (up to ~165 deg off observed),
        and since QuadPlane's own position controller converts
        world-frame desired motion into body-frame lean using the
        CURRENT heading, a badly wrong heading actively flies it away
        from the target until the YAW_RATE_MAX_DEG_S-capped correction
        catches up."""
        if self.fcu_mode != f'CMODE({MODE_AUTO})':
            return

        target_ned, _ = self._active_target()
        target_ned = target_ned + self._waypoint_offset

        d_north = target_ned[0] - self.pos[0]
        d_east = target_ned[1] - self.pos[1]
        if np.hypot(d_north, d_east) < 1.0:
            return  # at the target -- bearing is undefined/noisy this close, hold current yaw

        desired_yaw = float(np.arctan2(d_east, d_north))
        current_yaw = frames.yaw_from_quat(self.quat)
        yaw_err = float(np.arctan2(np.sin(desired_yaw - current_yaw),
                                    np.cos(desired_yaw - current_yaw)))

        yaw_rate_max_rad = np.radians(YAW_RATE_MAX_DEG_S)
        yaw_rate_cmd = float(np.clip(YAW_KP * yaw_err, -yaw_rate_max_rad, yaw_rate_max_rad))

        msg = OverrideRCIn()
        channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        channels[3] = pwm(yaw_rate_cmd / yaw_rate_max_rad, 1.0)
        msg.channels = channels
        self.rc_pub.publish(msg)

    def _tick(self):
        # Confirm the PREVIOUS push actually landed before allowing arm+AUTO
        # -- this used to be optimistic (mission_uploaded set the instant the
        # async request was FIRED, not when it was confirmed). If AUTO
        # engaged before the FCU had actually finished receiving the
        # mission, ArduPilot sees zero/partial items and instantly reports
        # "Mission complete, changing mode to RTL" -- confirmed as the real
        # mechanism behind every engage-sequence variant's failure tonight,
        # independent of which pre-arm hold mode was used.
        if self._mission_push_future is not None and self._mission_push_future.done():
            result = self._mission_push_future.result()
            self._mission_push_future = None
            if result is not None and result.success:
                self._mission_confirmed = True
                self.get_logger().info(f'mission confirmed ({result.wp_transfered} items)')
            else:
                self.mission_uploaded = False  # retry the push
                self.get_logger().warning('mission push failed or rejected, retrying')

        if not self.mission_uploaded:
            since_start = self.tick - SETPOINTS_BEFORE_OFFBOARD
            if since_start >= 0 and since_start % ENGAGE_RETRY_TICKS == 0:
                req = WaypointPush.Request()
                req.start_index = 0
                req.waypoints = self._build_mission()
                self._mission_push_future = self.mission_client.call_async(req)
                self.get_logger().info('requested mission upload')
                self.mission_uploaded = True

        was_armed = self._was_armed
        self._was_armed = self.mavros_armed

        if self._mission_complete:
            pass  # landed and disarmed once already -- hold, don't re-arm and refly
        elif not self.mavros_armed:
            if was_armed:
                # was armed last tick, not now -- that's a landing, not a
                # dropout (nothing else disarms this vehicle mid-mission).
                self._mission_complete = True
                self.get_logger().info('mission complete (landed and disarmed) -- holding')
                self._engage_phase = 'preflight'
            elif self._mission_confirmed and self._engage_phase == 'preflight':
                since_stream_started = self.tick - SETPOINTS_BEFORE_OFFBOARD - ENGAGE_RETRY_TICKS
                if (since_stream_started >= 0
                        and since_stream_started % ENGAGE_RETRY_TICKS == 0):
                    set_mode(self.cmd_client, MODE_AUTO)
                    arm(self.arm_client, True)
                    self.get_logger().info('requested AUTO mode and arm')
        elif self._engage_phase == 'preflight':
            self._engage_phase = 'auto'

        if self.mode == 'treatment':
            self._treatment_tick()

        self._yaw_to_target()

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
