"""Offboard setpoint streamer. Baseline mode publishes the raw mission
reference; treatment mode adds PINN feedforward behind the CBF filter.

The offboard arm/engage sequence follows the pattern already proven in
px4_teleop/teleop_keyboard.py: stream setpoints for a beat BEFORE
requesting offboard mode, because PX4 rejects the mode switch if no
setpoint stream is already present.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus,
                          VtolVehicleStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from . import canyon_geometry as cg
from . import constants as C
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .mission import Mission

SETPOINTS_BEFORE_OFFBOARD = 20  # PX4 needs an existing stream to accept the mode
ENGAGE_RETRY_TICKS = 50  # retry the arm+offboard request once a second until it sticks

# Tried again this session, live, twice, with a real fix attempt each
# time -- still disabled. What changed and what was learned:
#
# 1. Root-caused the original crash to PX4's own front-transition logic
#    (vtol_type.cpp isFrontTransitionCompletedBase) falling back to a
#    BLIND open-loop timer (VT_F_TR_OL_TM, 6s) whenever no airspeed
#    feedback is available, then ramming the rest of the tilt to
#    horizontal and cutting the rear hover motors in another 0.5s
#    (VT_TRANS_P2_DUR) regardless of the vehicle's actual state.
# 2. Tried enabling SYS_HAS_NUM_ASPD so PX4 could use real closed-loop
#    airspeed gating instead. model.sdf does declare an airspeed_link,
#    but nothing actually publishes on the Gazebo topic PX4's bridge
#    subscribes to in this build -- verified live, PX4's own console
#    logged "Preflight Fail: Airspeed invalid" and the vehicle never
#    armed at all (max altitude 12.5cm over an 82s run). Reverted; the
#    airspeed sensor isn't functional in this SITL setup, and making it
#    so is a separate, unstarted Gazebo-plugin task.
# 3. Fell back to PX4's blind open-loop timer, but added a
#    positive-climb-rate precondition here (on top of the existing speed
#    gate) before even requesting the transition, and set
#    VT_TILT_TRANS=0.5 (45 degrees) in the PX4-Autopilot airframe file --
#    the requested "gain speed and lift at a partial tilt, then once
#    climbing, finish the tilt" sequence, as closely as PX4's
#    architecture allows without a working airspeed sensor. Live-tested:
#    the vehicle no longer dives into the ground, but it never reaches
#    stable fixed-wing flight either -- once PX4's own P1/P2 state
#    machine takes over, it pitches wildly (0 to 88 degrees and back)
#    for the rest of the flight, horizontal speed collapses to
#    ~0.3-0.5 m/s, and it just drifts upward instead of flying the
#    canyon. Different failure shape, same underlying cause: the P1->P2
#    handoff (finish the tilt, cut the rear motors) is still a blind
#    clock, not a check on whether the wing is actually carrying the
#    vehicle's weight yet.
#
# Both of the levers actually reachable from this node (when to start,
# how far to tilt) are exhausted without a working airspeed signal. A
# real fix needs either (a) wiring the Gazebo airspeed sensor plugin so
# PX4's own closed-loop transition logic has real data to gate on, or
# (b) this node driving the tilt/throttle directly via
# /fmu/in/actuator_servos + /fmu/in/actuator_motors (OffboardControlMode
# .direct_actuator) instead of PX4's automatic state machine -- which
# means writing this project's own attitude stabilization for the
# transition phase, since direct_actuator bypasses PX4's rate/attitude
# controllers too. Both are real, separate pieces of work, not a
# trigger-condition or param tweak. Disabled again; the vehicle flies
# the whole transit in stable multicopter mode, which is how every
# verified-stable flight in this project's history has actually flown.
ENABLE_VTOL_TRANSITION = False
CLIMB_RATE_TRIGGER_MS = 0.3  # m/s upward (NED vz <= -this) required to request the transition

# How far past the canyon exit the vehicle must actually be before it
# lands, measured from the far edge of the LAST tower row (tower_2_n/
# tower_2_s), not from the mission's own exit waypoint (CANYON_EXIT is
# set generously past the towers -- 45m of margin -- for stable transit
# dynamics, not as a landing cue). local NED east = distance from
# CANYON_ENTRY (== the spawn point, verified live), so this is the last
# tower row's world-ENU edge shifted into that same local frame.
LAND_CLEARANCE_M = 2.0
_LAST_TOWER_EDGE_ENU_X = max(b.cx + b.sx / 2.0 for b in cg.BUILDINGS if b.cx > 0)
LAND_TRIGGER_LOCAL_M = _LAST_TOWER_EDGE_ENU_X - float(cg.CANYON_ENTRY[0]) + LAND_CLEARANCE_M

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


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')
        self.declare_parameter('mode', 'baseline')
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('baseline', 'treatment'):
            raise ValueError(f'mode must be baseline or treatment, got {self.mode}')
        self.get_logger().info(f'controller mode: {self.mode}')

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
        self.armed = False
        self.offboard_engaged = False
        self.vtol_transitioned = False
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

        self.sp_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.desired_pub = self.create_publisher(
            Vector3Stamped, C.TOPIC_SETPOINT_DESIRED, 10)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, qos_profile_sensor_data)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4',
            self._on_status, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_EST, self._on_wind_est, 10)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._on_attitude, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_TRUTH, self._on_wind_truth, 10)

        self.create_timer(1.0 / C.CONTROL_HZ, self._tick)

    def _on_position(self, msg):
        self.pos = np.array([msg.x, msg.y, msg.z])
        self.vel = np.array([msg.vx, msg.vy, msg.vz])

    def _on_status(self, msg):
        self.armed = msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        self.offboard_engaged = msg.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD

    def _on_wind_est(self, msg):
        self.wind_est = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _on_attitude(self, msg):
        self.quat = np.array([msg.q[0], msg.q[1], msg.q[2], msg.q[3]])

    def _on_wind_truth(self, msg):
        self.wind_truth = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = True
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.mode_pub.publish(msg)

    def _tick(self):
        if self.land_requested:
            # Handed off to PX4's own AUTO_LAND -- continuing to publish
            # an offboard setpoint stream here would fight it for control
            # authority the moment nav_state leaves OFFBOARD.
            self.tick += 1
            return

        self._publish_offboard_mode()

        engaged = self.armed and self.offboard_engaged
        since_stream_started = self.tick - SETPOINTS_BEFORE_OFFBOARD
        if (not engaged and since_stream_started >= 0
                and since_stream_started % ENGAGE_RETRY_TICKS == 0):
            # A single request can be silently rejected if PX4 hasn't
            # finished its own preflight/EKF convergence yet -- keep
            # asking once a second until the vehicle actually confirms
            # armed + offboard, rather than trying exactly once and
            # leaving the vehicle idle for the rest of the trial with no
            # visible error.
            # 1 = custom main mode, 6 = offboard
            self._send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self._send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            self.get_logger().info('requested offboard mode and arm')

        if engaged and self.start_time is None:
            self.start_time = self.get_clock().now()

        elapsed = 0.0
        if self.start_time is not None:
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        if (ENABLE_VTOL_TRANSITION and engaged and not self.vtol_transitioned
                and elapsed >= self.mission.hold_s):
            # The hold phase (elapsed < hold_s, target pinned at
            # CANYON_ENTRY) is the vertical climb to altitude -- takeoff.
            # Transitioning to fixed-wing right at elapsed == hold_s
            # (originally gated on elapsed alone, for determinism) stalled
            # and crashed the vehicle every time: that instant is exactly
            # when the target has JUST started moving off the pinned hold
            # point, so horizontal speed is still ~0 -- a VTOL has no lift
            # in FW mode without forward airspeed. Gate on measured ground
            # speed reaching a safe margin below cruise instead, with a
            # generous elapsed-time cap so a persistently low reading
            # (e.g. stuck telemetry) can't withhold the transition forever.
            speed = float(np.linalg.norm(self.vel[:2]))
            climbing = self.vel[2] <= -CLIMB_RATE_TRIGGER_MS
            if ((speed >= 0.8 * self.mission.speed and climbing)
                    or elapsed >= self.mission.hold_s + 15.0):
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
                                   float(VtolVehicleStatus.VEHICLE_VTOL_STATE_FW))
                self.vtol_transitioned = True
                self.get_logger().info(
                    f'requested VTOL transition to fixed-wing at '
                    f'speed={speed:.1f} m/s vz={self.vel[2]:.1f} m/s')

        target, done = self.mission.target(elapsed)
        if done and not self.done_logged:
            self.get_logger().info(f'mission complete at t={elapsed:.1f}s')
            self.done_logged = True

        # self.pos[1] is local NED east -- since the vehicle spawns at
        # CANYON_ENTRY (run_trial.SPAWN_XYZ), PX4's local origin sits
        # there too (verified live), so self.pos[1] already reads as
        # "east distance travelled from the entry" directly, and
        # LAND_TRIGGER_LOCAL_M is that same distance to the last tower
        # row's far edge plus LAND_CLEARANCE_M. Gated on measured
        # position, not the open-loop mission schedule's `done` flag, so
        # landing only fires once the vehicle has actually cleared the
        # towers -- wind or CBF deviation could otherwise have it trigger
        # while still between the buildings.
        if engaged and self.pos[1] >= LAND_TRIGGER_LOCAL_M:
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.land_requested = True
            self.get_logger().info(
                f'cleared the tower row by {LAND_CLEARANCE_M}m -- requested landing')
            self.tick += 1
            return

        sp = TrajectorySetpoint()
        sp.position = [float(v) for v in target]
        # TrajectorySetpoint.msg: "setting a value to NaN means the state
        # should not be controlled". velocity/acceleration default to
        # [0.0, 0.0, 0.0], NOT NaN -- left alone, that is read by PX4 as an
        # explicit hold-zero-velocity/zero-acceleration command layered on
        # top of the position setpoint, fighting the position controller's
        # own authority to move the vehicle. Verified live: with this
        # unset, cruise velocity never got anywhere near the mission's
        # 12 m/s (capped around 2-6 m/s), and a large reverse position
        # setpoint (tried in an earlier fly-home-and-land design -- see
        # LAND_CLEARANCE_M above) produced no turnaround at all -- the
        # vehicle just kept drifting the same direction it was already
        # moving. Explicitly marking both NaN is what actually hands full
        # authority to the position controller.
        sp.velocity = [float('nan')] * 3
        sp.acceleration = [float('nan')] * 3
        sp.yaw = self.cruise_yaw  # nose down the canyon's actual travel direction

        if self.mode == 'treatment':
            # The PINN estimates the disturbance FORCE; the feedforward is
            # the acceleration that cancels it. Negative: we push back.
            # Only reached during the actual transit -- landing hands off
            # to AUTO_LAND above and returns before this point.
            u_des = -self.wind_est / MASS_KG
            u_safe, info = self.cbf.filter(
                u_des, self.pos, self.vel, self.wind_truth, self.quat)
            sp.acceleration = [float(v) for v in u_safe]

            diag = Vector3Stamped()
            diag.header.stamp = self.get_clock().now().to_msg()
            diag.vector.x = 1.0 if info['active'] else 0.0
            # Obstacle barrier only (metres) -- see cbf_filter.py for why
            # this must never be combined with the stall barrier (radians).
            diag.vector.y = float(np.clip(info['h_obstacle'], -1e3, 1e3))
            diag.vector.z = 0.0 if info['feasible'] else 1.0
            self.cbf_pub.publish(diag)

        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.sp_pub.publish(sp)

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
