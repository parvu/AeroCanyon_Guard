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

from . import constants as C
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .mission import Mission

SETPOINTS_BEFORE_OFFBOARD = 20  # PX4 needs an existing stream to accept the mode
ENGAGE_RETRY_TICKS = 50  # retry the arm+offboard request once a second until it sticks

# Requesting DO_VTOL_TRANSITION, even gated on measured forward speed
# reaching 80% of cruise, produced a violent, self-reinforcing pitch
# oscillation in live testing: a sudden climb (vz jumping to +8 m/s),
# then a stall, then a drop down to ~2 m above the ground before
# partially recovering. This is a real PX4 VTOL attitude/airspeed-control
# tuning problem (transition ramp rate, FW pitch/airspeed gains), not a
# logic bug fixable by adjusting the trigger condition -- it needs
# careful iterative tuning against live telemetry, not a blind parameter
# guess. Disabled until that tuning happens; the vehicle instead flies
# the whole transit in stable multicopter mode, which is how every
# verified-stable flight in this project's history has actually flown.
ENABLE_VTOL_TRANSITION = False

# How far past the canyon exit (measured position, not the open-loop
# mission schedule) the vehicle must actually be before RTL is requested.
RTL_CLEARANCE_M = 2.0

# VEHICLE_CMD_NAV_RETURN_TO_LAUNCH does engage nav_state=AUTO_RTL (verified
# live: vtol_vehicle_status stayed MC throughout, ruling out a repeat of
# the VTOL-transition instability above), but in this SITL configuration
# it does not actually navigate the vehicle back toward home -- verified
# live over a full flight: local east kept increasing in the same
# direction the whole time, all the way out to ~1900 m (versus the ~200 m
# canyon), never turning back, right through PX4's own 60 m RTL_RETURN_ALT
# climb. This looks like a home-position problem (COM_ARM_WO_GPS=1 may be
# skipping normal home-position initialisation), not something a trigger
# condition can fix, and not safe to leave running unattended while
# unexplained. Disabled until root-caused; the vehicle instead keeps
# flying the open-loop mission (hovering at the final target once
# Mission.target() reports done) with no attempt at an autonomous return.
ENABLE_RTL_ON_CLEARANCE = False


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
        self.rtl_requested = False
        self.done_logged = False
        self.wind_est = np.zeros(3)

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

        self.cbf = CBFFilter()
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
        if self.rtl_requested:
            # Hand off entirely to PX4's own RTL/land logic. Continuing to
            # publish an offboard setpoint stream here would fight it for
            # control authority the moment nav_state leaves OFFBOARD --
            # this node's job is done once it's asked to go home.
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
            if speed >= 0.8 * self.mission.speed or elapsed >= self.mission.hold_s + 15.0:
                self._send_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
                                   float(VtolVehicleStatus.VEHICLE_VTOL_STATE_FW))
                self.vtol_transitioned = True
                self.get_logger().info(
                    f'requested VTOL transition to fixed-wing at speed={speed:.1f} m/s')

        target, done = self.mission.target(elapsed)
        if done and not self.done_logged:
            self.get_logger().info(f'mission complete at t={elapsed:.1f}s')
            self.done_logged = True

        # self.pos[1] is local NED east -- since the vehicle spawns at
        # CANYON_ENTRY (run_trial.SPAWN_XYZ), PX4's local origin sits
        # there too (verified live), so self.pos[1] already reads as
        # "east distance travelled from the entry" directly, and
        # mission.distance is exactly that distance to the exit. Gated on
        # measured position, not the open-loop mission schedule's `done`
        # flag, so RTL only fires once the vehicle has actually cleared
        # the canyon -- wind or CBF deviation could otherwise have it
        # trigger while still between the buildings.
        if (ENABLE_RTL_ON_CLEARANCE and engaged
                and self.pos[1] >= self.mission.distance + RTL_CLEARANCE_M):
            self._send_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
            self.rtl_requested = True
            self.get_logger().info(
                f'cleared canyon by {RTL_CLEARANCE_M}m -- requested RTL')
            self.tick += 1
            return

        sp = TrajectorySetpoint()
        sp.position = [float(v) for v in target]
        sp.yaw = self.cruise_yaw  # nose down the canyon's actual travel direction

        if self.mode == 'treatment':
            # The PINN estimates the disturbance FORCE; the feedforward is
            # the acceleration that cancels it. Negative: we push back.
            u_des = -self.wind_est / MASS_KG
            u_safe, info = self.cbf.filter(
                u_des, self.pos, self.vel, self.wind_truth, self.quat)
            sp.acceleration = [float(v) for v in u_safe]

            diag = Vector3Stamped()
            diag.header.stamp = self.get_clock().now().to_msg()
            diag.vector.x = 1.0 if info['active'] else 0.0
            diag.vector.y = float(np.clip(info['h_min'], -1e3, 1e3))
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
