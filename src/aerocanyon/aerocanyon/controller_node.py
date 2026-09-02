"""Offboard setpoint streamer. Baseline mode publishes the raw mission
reference; treatment mode adds PINN feedforward behind the CBF filter.

The offboard arm/engage sequence follows the pattern already proven in
px4_teleop/teleop_keyboard.py: stream setpoints for a beat BEFORE
requesting offboard mode, because PX4 rejects the mode switch if no
setpoint stream is already present.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, CommandLong
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from . import canyon_geometry as cg
from . import constants as C
from . import frames
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .mission import Mission
from .rc_pwm import (MODE_QHOVER, MODE_QLAND, arm, pwm, pwm_throttle,
                     resolve_stick, set_mode)

SETPOINTS_BEFORE_OFFBOARD = 20  # PX4 needs an existing stream to accept the mode
ENGAGE_RETRY_TICKS = 50  # retry the arm+offboard request once a second until it sticks

# Tried repeatedly this session, live, with real fix attempts each round
# -- still disabled. What changed and what was learned, in order:
#
# 1. Root-caused the original crash to PX4's own front-transition logic
#    (vtol_type.cpp isFrontTransitionCompletedBase) falling back to a
#    BLIND open-loop timer (VT_F_TR_OL_TM, 6s) whenever no airspeed
#    feedback is available, then ramming the rest of the tilt to
#    horizontal and cutting the rear hover motors in another 0.5s
#    (VT_TRANS_P2_DUR) regardless of the vehicle's actual state.
# 2. Enabled SYS_HAS_NUM_ASPD so PX4 could use real closed-loop airspeed
#    gating instead. First attempt failed: model.sdf declares an
#    airspeed_link, but nothing was publishing on the Gazebo topic PX4's
#    bridge subscribes to -- "Preflight Fail: Airspeed invalid", vehicle
#    never armed. Root cause (found by reading the world file): the
#    world's plugin list loads gz-sim-imu-system, air-pressure-system,
#    magnetometer-system, and navsat-system explicitly, but never
#    gz-sim-air-speed-system -- each sensor TYPE needs its own system
#    plugin to actually compute and publish data, and that one was just
#    missing. Added it (worlds/_template.sdf, regenerated into
#    worlds/urban_canyon.sdf and copied to PX4-Autopilot). Verified live:
#    PX4's console now logs "Airspeed sensor healthy", arming succeeds,
#    and a full run with the climb-rate-gated trigger + VT_TILT_TRANS=0.5
#    actually completed the canyon transit and landed safely -- the first
#    time any transition attempt this session did that. This part is a
#    real, kept fix: the airspeed sensor is genuine, reusable
#    infrastructure regardless of what happens with the transition
#    itself (SYS_HAS_NUM_ASPD stays 1 in the airframe file).
# 3. That successful run still wasn't smooth: recurring pitch excursions
#    during cruise (past +-45 degrees, briefly +-80, self-recovering).
#    Suspected TECS chasing an unreachable airspeed -- PX4's stock
#    FW_AIRSPD_TRIM=15/MIN=10 m/s were never overridden, but this
#    vehicle only ever reaches ~7-11 m/s. Tried FW_AIRSPD_STALL=6/MIN=7/
#    TRIM=10/MAX=15. Live-tested: WORSE, not better -- instead of a
#    bounded wobble around cruise altitude, the vehicle entered a
#    sustained ~22s nose-down dive (pitch to -87 degrees, sink rate up to
#    7.7 m/s) from 25m almost to ground contact before barely arresting
#    it. Reverted; the performance-model numbers (trim/stall/climb/sink)
#    aren't self-consistent for this vehicle and guessing at them further
#    risks a real crash rather than fixing the wobble.
#
# Net position: the airspeed sensor fix (2) is real and kept. The
# transition can now complete a full flight without diving or getting
# stuck, but cruise is still rough, and the one attempt to smooth it (3)
# made things actively worse. This needs proper TECS/attitude-rate
# tuning against live telemetry by someone who can iterate on it
# carefully -- not another guessed parameter set -- or the direct-
# actuator alternative (this node driving tilt/throttle itself via
# /fmu/in/actuator_servos + /fmu/in/actuator_motors with
# OffboardControlMode.direct_actuator, replacing PX4's state machine
# entirely, which also means writing this project's own attitude
# stabilization for the transition phase). Disabled for now; the vehicle
# flies the whole transit in stable multicopter mode, which is how every
# verified-stable flight in this project's history has actually flown.
ENABLE_VTOL_TRANSITION = False

# How far past the canyon exit the vehicle must actually be before it
# lands, measured from the far edge of the LAST tower row (tower_2_n/
# tower_2_s), not from the mission's own exit waypoint (CANYON_EXIT is
# set generously past the towers -- 45m of margin -- for stable transit
# dynamics, not as a landing cue).
#
# self.pos is in the SAME absolute canyon-frame NED coordinates as
# mission.target()/canyon_geometry.BUILDINGS, not relative to the
# vehicle's own spawn point -- confirmed live by querying
# LOCAL_POSITION_NED directly at spawn, unarmed, before any control
# input: it already reads (~0, ~-100, ~0), matching CANYON_ENTRY's own
# NED value exactly, not (0,0,0). (An earlier version of this comment,
# and of the code below, assumed the opposite -- that self.pos was
# spawn-relative, the way a placeholder-zero row at the very start of a
# trial_logger CSV can look before any real telemetry has arrived -- and
# both were wrong; reverted.) So the threshold below is a plain absolute
# NED-east value: the last tower row's world-ENU edge (NED-east ==
# ENU-east, see frames.py) plus clearance, no CANYON_ENTRY subtraction.
LAND_CLEARANCE_M = 2.0
_LAST_TOWER_EDGE_ENU_X = max(b.cx + b.sx / 2.0 for b in cg.BUILDINGS if b.cx > 0)
LAND_TRIGGER_LOCAL_M = _LAST_TOWER_EDGE_ENU_X + LAND_CLEARANCE_M

# Earlier designs tried flying the vehicle all the way back to the spawn
# point before landing via native RTL (VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
# engages AUTO_RTL correctly but, verified live, never actually navigates
# back toward home in this SITL configuration, drifting to ~1900m instead
# of turning around). That complexity existed to solve one problem: a
# vehicle left drifting or crashed when the next leg's PX4 process
# booted, since Gazebo and the vehicle entity used to stay alive across
# both legs. Now that each leg gets its own fresh `gz sim` + PX4 process
# (run_trial.run_leg) with nothing shared between legs at all, THAT
# problem no longer exists -- wherever this leg's vehicle ends up is
# irrelevant to the next leg's boot. So: just land in place, handed off
# to PX4's own VEHICLE_CMD_NAV_LAND / AUTO_LAND.
#
# A version of this node tried descending under its OWN offboard control
# instead (freezing the position at the clearance point, target z=0)
# specifically to keep the heading locked -- AUTO_LAND was verified live
# to visibly turn the vehicle during the descent, off whatever heading it
# had at clearance. That self-controlled descent held heading correctly,
# but its own disarm logic (needed since nothing else would ever stop it)
# proved unsafe: verified live, repeatedly, that the vehicle could
# destabilise into a violent, uncontrolled tumble -- most likely because
# VEHICLE_CMD_COMPONENT_ARM_DISARM without PX4's force parameter can be
# silently REJECTED while airborne, and this node stopped publishing
# setpoints the moment it (wrongly) believed the disarm had succeeded,
# leaving the vehicle under thrust with no control input at all. AUTO_LAND
# -- PX4's own, extensively field-tested landing logic, including its own
# correct handling of when disarming is actually safe -- doesn't have
# that failure mode. A turn during descent is a cosmetic issue; loss of
# control is not, so this hands off to AUTO_LAND unconditionally now,
# heading be damned.


def _yaw_from_quat(q):
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('mode', 'baseline')
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('baseline', 'treatment'):
            raise ValueError(f'mode must be baseline or treatment, got {self.mode}')
        self.get_logger().info(f'controller mode: {self.mode}')

        # Scales the PINN feedforward before it reaches the CBF. 1.0 is the
        # original behaviour: cancel the entire estimated wind force. Measured
        # in flight, that commands |wind_est|/m = 6.5 m/s^2 mean, 13.5 m/s^2 at
        # p95 -- against PX4's ~3 m/s^2 horizontal acceleration budget, and
        # comparable to gravity. Such a correction swamps the position
        # controller rather than trimming it, which is how an estimator with
        # genuine open-loop skill (0.665 in flight) still produced no closed-
        # loop benefit: mean improvement +0.017 m over n=8 seeds, p=0.98.
        # Part of the oversizing is structural -- wind_force() returns the
        # TOTAL aerodynamic force, including the drag and lift the vehicle
        # makes flying through still air, which is not a disturbance to cancel.
        #
        # Subtracting that still-air force was tried as the principled fix and
        # MEASURED not to work: it is only 18% of the total, and because the
        # two vectors partly oppose, removing it makes the feedforward LARGER
        # (56.3 N vs 51.6 N). The oversizing is not about still-air terms --
        # the modelled aero force is simply ~10 m/s^2, about 1 g, most of which
        # PX4's position-control FEEDBACK is already rejecting (baseline flies
        # fine). Feeding all of it forward double-counts the feedback loop.
        # 0.2 is what fits inside the controller's authority: 0.2 * 10.08 =
        # 2.0 m/s^2 against MPC_ACC_HOR's 3.0. Measured over 8 paired seeds,
        # gain 1.0 gave +0.4% (p=0.98) and gain 0.2 gave +21.1% (p=0.12).
        self.declare_parameter('feedforward_gain', 0.2)
        self.ff_gain = float(self.get_parameter('feedforward_gain').value)

        self.mission = Mission()
        # NED yaw (0 = north, +pi/2 = east) pointing down the canyon's
        # actual travel direction -- do NOT hardcode this to 0.0. The
        # canyon corridor runs along Gazebo ENU +x (east, see
        # canyon_geometry.BUILDINGS), not north, so a fixed yaw=0.0 here
        # previously pointed the nose north while the mission pulled the
        # vehicle east: the vehicle would take off facing the wrong way
        # and visibly snap ~90 degrees once flight caught up to it.
        self.cruise_yaw = float(np.arctan2(
            self.mission.direction[1], self.mission.direction[0]))
        self.tick = 0
        self.start_time = None
        self.land_requested = False
        self.done_logged = False
        self.wind_est = np.zeros(3)

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

        # The CBF's stall barrier is a fixed-wing (loss-of-lift) concept --
        # only meaningful while the vehicle is actually flying fixed-wing,
        # i.e. tied to the same flag as the VTOL transition itself. See
        # cbf_filter.py's module docstring for why leaving it on
        # unconditionally corrupted the safety diagnostic.
        self.cbf = CBFFilter(enable_stall=ENABLE_VTOL_TRANSITION)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.wind_truth = np.zeros(3)
        self.cbf_pub = self.create_publisher(
            Vector3Stamped, C.TOPIC_CBF_DIAG, 10)

        self.rc_pub = self.create_publisher(OverrideRCIn, '/mavros/rc/override', 10)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.cmd_client = self.create_client(CommandLong, '/mavros/cmd/command')
        self.desired_pub = self.create_publisher(
            Vector3Stamped, C.TOPIC_SETPOINT_DESIRED, 10)

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

    @staticmethod
    def _lean_from_accel(accel_ned, yaw_quat):
        """Desired horizontal NED acceleration -> (roll, pitch) body-frame
        lean angles, small-angle. Rotates the horizontal acceleration into
        the body frame using current yaw only (not full 3D attitude --
        QHOVER's own attitude controller handles roll/pitch response to
        this lean command; this loop only needs to know which way is
        'forward' right now)."""
        g = C.G
        yaw = _yaw_from_quat(yaw_quat)
        ax_body = accel_ned[0] * math.cos(yaw) + accel_ned[1] * math.sin(yaw)
        ay_body = -accel_ned[0] * math.sin(yaw) + accel_ned[1] * math.cos(yaw)
        pitch = math.atan2(ax_body, g)
        roll = math.atan2(ay_body, g)
        return roll, pitch

    def _tick(self):
        if self.land_requested:
            # Handed off to ArduPilot's own QLAND -- continuing to publish
            # RC overrides here would fight it for control authority.
            self.tick += 1
            return

        engaged = self.mavros_armed
        since_stream_started = self.tick - SETPOINTS_BEFORE_OFFBOARD
        if (not engaged and since_stream_started >= 0
                and since_stream_started % ENGAGE_RETRY_TICKS == 0):
            # A single request can be silently rejected if the EKF hasn't
            # converged yet -- keep asking once a second until MAVROS
            # actually confirms armed, rather than trying exactly once and
            # leaving the vehicle idle for the rest of the trial with no
            # visible error.
            set_mode(self.cmd_client, MODE_QHOVER)
            arm(self.arm_client, True)
            self.get_logger().info('requested QHOVER mode and arm')

        if engaged and self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        target, done = self.mission.target(elapsed)
        if done and not self.done_logged:
            self.get_logger().info(f'mission complete at t={elapsed:.1f}s')
            self.done_logged = True

        # self.pos[1] is local NED east -- since the vehicle spawns at
        # CANYON_ENTRY (run_trial.SPAWN_XYZ), the local origin sits there
        # too (verified live under PX4; unchanged assumption here), so
        # self.pos[1] already reads as "east distance travelled from the
        # entry" directly, and LAND_TRIGGER_LOCAL_M is that same distance
        # to the last tower row's far edge plus LAND_CLEARANCE_M. Gated on
        # measured position, not the open-loop mission schedule's `done`
        # flag, so landing only fires once the vehicle has actually
        # cleared the towers -- wind or CBF deviation could otherwise have
        # it trigger while still between the buildings.
        if engaged and self.pos[1] >= LAND_TRIGGER_LOCAL_M:
            set_mode(self.cmd_client, MODE_QLAND)
            self.land_requested = True
            self.get_logger().info(
                f'cleared the tower row by {LAND_CLEARANCE_M}m -- requested landing')
            self.tick += 1
            return

        if self.mode == 'baseline':
            # PD toward the mission target -- PX4's own position
            # controller used to compute this; ArduPilot exposes no
            # equivalent injection path for this airframe (no MAVLink XY
            # position/velocity setpoint works for any Q-mode here), so
            # this project now provides it. self.pos is already in the
            # same absolute canyon-frame NED coordinates as target -- see
            # LAND_TRIGGER_LOCAL_M's comment above.
            pos_err = target - self.pos
            u_des = C.POSITION_KP * pos_err - C.POSITION_KD * self.vel
        else:
            # The PINN estimates the disturbance FORCE; the feedforward is
            # the acceleration that cancels it. Negative: we push back.
            u_des = -self.ff_gain * self.wind_est / MASS_KG

        u_safe, info = self.cbf.filter(u_des, self.pos, self.vel,
                                       self.wind_truth, self.quat)

        if self.mode == 'treatment':
            diag = Vector3Stamped()
            diag.header.stamp = self.get_clock().now().to_msg()
            diag.vector.x = 1.0 if info['active'] else 0.0
            # Obstacle barrier only (metres) -- see cbf_filter.py for why
            # this must never be combined with the stall barrier (radians).
            diag.vector.y = float(np.clip(info['h_obstacle'], -1e3, 1e3))
            diag.vector.z = 0.0 if info['feasible'] else 1.0
            self.cbf_pub.publish(diag)

        roll, pitch = self._lean_from_accel(u_safe[:2], self.quat)
        alt_err = target[2] - self.pos[2]  # NED: more negative target = higher
        climb_cmd = float(np.clip(-C.ALTITUDE_KP * alt_err, -1.0, 1.0))
        heading_err = _wrap_pi(self.cruise_yaw - _yaw_from_quat(self.quat))
        yaw_cmd = float(np.clip(C.HEADING_KP * heading_err, -1.0, 1.0))

        msg = OverrideRCIn()
        channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        channels[0:4] = [
            pwm(float(np.clip(roll / C.MAX_LEAN_RAD, -1.0, 1.0)), 1.0),
            pwm(float(np.clip(pitch / C.MAX_LEAN_RAD, -1.0, 1.0)), 1.0, invert=True),
            pwm_throttle(climb_cmd, 1.0),
            pwm(yaw_cmd, 1.0),
        ]
        channels[4:8] = [1500] * 4
        msg.channels = channels
        self.rc_pub.publish(msg)

        desired = Vector3Stamped()
        desired.header.stamp = self.get_clock().now().to_msg()
        desired.vector.x, desired.vector.y, desired.vector.z = (
            float(target[0]), float(target[1]), float(target[2]))
        self.desired_pub.publish(desired)

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
