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
from gz.transport13 import Node as GzNode
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleAttitude, VehicleCommand,
                          VehicleLocalPosition, VehicleStatus)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from . import canyon_geometry as cg
from . import catapult
from . import constants as C
from .cbf_filter import CBFFilter
from .constants import MASS_KG
from .mission import Mission

SETPOINTS_BEFORE_OFFBOARD = 20  # PX4 needs an existing stream to accept the mode
ENGAGE_RETRY_TICKS = 50  # retry the arm+offboard request once a second until it sticks

# This is the wing-only branch: pure fixed-wing, catapult-launched, no hover
# capability at all. The VTOL-transition machinery that lived here on the
# tricopter/tiltrotor branches (ENABLE_VTOL_TRANSITION and its whole history
# of attempted fixes) is gone -- not applicable, there is no MC mode on this
# airframe to transition FROM. See catapult.py for how the launch itself is
# simulated, and its module docstring for why PX4's own FW_LAUN_DETCN_ON
# launch detector can't be used from this project's OFFBOARD architecture.
#
# STATUS, verified live across several rounds:
#
# 1. Catapult launch mechanism: works, but took two real bugs to get there.
#    (a) The static motor-nacelle visual (models/wingonly/model.sdf's
#        motor_rear) was never rotated to match rotor_1's real horizontal
#        thrust axis -- it stood upright as if for a vertical hover motor.
#        Fixed, and the tail boom (a leftover from the tricopter's offset
#        tail rotor, not needed for a fuselage-mounted pusher) removed.
#    (b) catapult.start()/stop() originally called gz_node.advertise()
#        immediately before the first publish -- a gz-transport discovery
#        race that can silently drop that first message. Measured directly:
#        one run's launch acceleration came out ~100x smaller than
#        intended, consistent with the toss being lost and only
#        FW_THR_IDLE creep showing up. Fixed by advertising once at node
#        startup (catapult.Launcher) instead of at launch time.
# 2. Motor thrust: was 2e-05 motorConstant, copied unchanged from the
#    tricopter where it was one of THREE rotors sharing hover load. As the
#    SOLE forward-thrust source here, max available thrust at FW_THR_MAX
#    was ~16N against this airframe's ~49N weight -- 33%, nowhere near
#    enough climb authority. Set to 3x (6e-05, ~99% of weight) -- a real,
#    computed fix, kept regardless of (3) below.
# 3. Sustained FW flight control: STILL UNRESOLVED, confirmed present even
#    with (1) and (2) both fixed -- this rules out "it just needs a
#    working launch and more thrust" as the explanation. Live-verified: a
#    clean, fast launch pulse (0 to ~6 m/s within the 0.3s stroke, not
#    smeared over seconds) is followed by steady deceleration back to a
#    dead stop, still essentially on the ground (max altitude 4cm), well
#    short of the mission's 25m cruise target. Same class of problem the
#    tricopter/tiltrotor hit in FW mode (see History.md) -- copied
#    reference rate-loop gains get the vehicle moving, not tracking.
#    Root cause not yet diagnosed (candidates: TECS altitude gate, an
#    offboard message field fighting position control the way an unset
#    velocity/acceleration field did for the tricopter, or the
#    FW_POSCTRL mode never actually engaging for this airframe). Do not
#    assume position/altitude tracking works on this branch without
#    re-verifying live.

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

        # hold_s=0: the tricopter/tiltrotor's hold phase pinned the vehicle
        # over one spot to climb vertically before starting forward travel --
        # meaningless for a fixed-wing airframe with no hover capability at
        # all. This vehicle needs to be moving from the very first tick.
        self.mission = Mission(hold_s=0.0)
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
        self.land_requested = False
        self.done_logged = False
        self.wind_est = np.zeros(3)

        self.pos = np.zeros(3)
        self.vel = np.zeros(3)

        # Catapult toss (see catapult.py): fired once, on the first tick the
        # vehicle is armed and offboard-engaged. gz.transport13's Node is
        # this project's existing pattern for talking to Gazebo directly
        # (run_trial.py's _reset_gazebo_model uses the same client type).
        # The Launcher advertises its publishers HERE, at node startup --
        # not right before the time-critical first publish -- see
        # catapult.py's module docstring for why that matters (a discovery
        # race that silently dropped the toss, live-verified).
        self.gz = GzNode()
        self.launcher = catapult.Launcher(self.gz, C.WORLD_NAME, C.MODEL_NAME)
        self.launched = False
        self.launch_time = None
        self._launch_force_n = catapult.force_newtons(MASS_KG)

        # This airframe is ALWAYS flying fixed-wing -- unlike the tricopter,
        # where the stall barrier was tied to the (disabled) VTOL transition
        # flag because MC flight has no meaningful angle of attack, this
        # vehicle has no MC mode to guard against. Enabled unconditionally.
        self.cbf = CBFFilter(enable_stall=True)
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

        if engaged and not self.launched:
            # Fires exactly once, the instant the vehicle is armed and
            # offboard-engaged -- see catapult.py for why this simulates the
            # toss directly (a real force on the airframe) instead of going
            # through PX4's own launch-detection module.
            self.launcher.start(self._launch_force_n)
            self.launched = True
            self.launch_time = self.get_clock().now()
            self.get_logger().info(
                f'catapult: applying {self._launch_force_n:.0f} N for '
                f'{catapult.LAUNCH_DURATION_S:.1f}s')
        elif self.launched and self.launch_time is not None:
            since_launch = (self.get_clock().now() - self.launch_time).nanoseconds / 1e9
            if since_launch >= catapult.LAUNCH_DURATION_S:
                self.launcher.stop()
                self.launch_time = None  # one-shot: never call stop() again
                self.get_logger().info('catapult: released')

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
            u_des = -self.ff_gain * self.wind_est / MASS_KG
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
