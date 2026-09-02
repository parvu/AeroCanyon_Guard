"""Replay the canyon wind field into Gazebo at the vehicle's position.

Gazebo's WindEffects plugin models a GLOBALLY UNIFORM wind. Rather than
write a new plugin, this node looks the field up at the vehicle's position
and republishes that as the global wind at CONTROL_HZ. A single vehicle
therefore experiences the correct spatially varying field.

ponytail: single-vehicle only -- the global wind topic is driven from one
drone's position. Multi-vehicle needs a real per-link wind plugin.

ros_gz_interfaces has no Wind message, so this cannot go through
parameter_bridge; the node publishes to Gazebo directly via gz.transport13.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from gz.msgs10.wind_pb2 import Wind
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from . import constants as C
from . import frames
from .canyon_field import DrydenGust, WindGrid


def cap_speed(vec, max_speed):
    """Clamp a vector's magnitude to max_speed, direction unchanged.
    Pure function so it's testable without the rclpy node -- see
    MC_MAX_WIND_SPEED_MS's own comment for why this exists."""
    mag = np.linalg.norm(vec)
    if mag <= max_speed or mag < 1e-9:
        return vec
    return vec * (max_speed / mag)


# MC (hover/VTOL) flight target wind speed, m/s -- caps the STEADY
# mean-flow component only (gusts stay at their own turbulence_sigma,
# same as wind_speed_scale above). Deliberately does NOT touch
# self.grid itself, whose own data averages ~8.8 m/s across the canyon
# (see wind_field_node's module docstring/History.md) -- that full-
# strength field is left intact for a future fixed-wing flight phase
# (not yet implemented -- ENABLE_VTOL_TRANSITION stays False in
# controller_node.py) to read directly, uncapped, once it exists.
MC_MAX_WIND_SPEED_MS = 2.0


class WindFieldNode(Node):

    def __init__(self):
        super().__init__('wind_field_node')
        self.declare_parameter('data_dir', '')
        # Dryden gust intensity. Raised from the original 1.5 to 4.0 after
        # measuring what the vehicle actually experienced: the disturbance was
        # dominated by the STEADY mean-flow grid (mean 8.8 m/s, per-axis std
        # 0.99/4.75/1.48), most of which PX4's own position-controller
        # integrator absorbs by itself. That left almost nothing unsteady for
        # a feedforward wind estimate to contribute, and the baseline/treatment
        # difference sat below the run-to-run noise floor (see History.md).
        # Gusts are the component a FO-PINN feedforward can uniquely help with,
        # so they are what needs to be representative of a real urban canyon.
        self.declare_parameter('turbulence_sigma', 2.5)
        self.declare_parameter('seed', 0)
        self.declare_parameter('mc_max_wind_speed_ms', MC_MAX_WIND_SPEED_MS)

        data_dir = self.get_parameter('data_dir').value
        if not data_dir:
            from ament_index_python.packages import get_package_share_directory
            data_dir = get_package_share_directory('aerocanyon') + '/data'
        self.grid = WindGrid.load(data_dir)
        self.get_logger().info(f'loaded wind grid from {data_dir}')

        dt = 1.0 / C.CONTROL_HZ
        self.gust = DrydenGust(
            dt=dt,
            sigma=float(self.get_parameter('turbulence_sigma').value),
            seed=int(self.get_parameter('seed').value),
        )

        self.pos_enu = np.zeros(3)
        self.airspeed = 1.0
        # Live-adjustable multiplier on the STEADY mean-flow grid lookup
        # only -- gusts (self.gust, above) stay at their own configured
        # turbulence_sigma regardless. Real request: the browser's
        # spd+/spd- buttons ("wind medium speed") drive this, via
        # /aerocanyon/wind_speed_scale -- see control_server.py's
        # --no-rc mode, which is what actually runs alongside an
        # autonomous run_trial.py leg (the normal RC-publishing mode
        # would fight controller_node for /mavros/rc/override).
        self.wind_speed_scale = 1.0
        self.mc_max_wind_speed_ms = float(
            self.get_parameter('mc_max_wind_speed_ms').value)
        self.create_subscription(
            Float64, C.TOPIC_WIND_SPEED_SCALE, self._on_speed_scale, 10)

        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._on_pose, qos_profile_sensor_data)
        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, qos_profile_sensor_data)

        self.truth_pub = self.create_publisher(
            Vector3Stamped, C.TOPIC_WIND_TRUTH, 10)

        self.gz = GzNode()
        self.gz_pub = self.gz.advertise(C.GZ_WIND_TOPIC, Wind)

        self.create_timer(dt, self._tick)

    def _on_pose(self, msg):
        # MAVROS publishes local_position in ENU directly (ROS convention)
        # -- the grid is also ENU, so no frame conversion needed here at
        # all (unlike controller_node.py, which converts to NED for its
        # own NED-frame math).
        p = msg.pose.position
        self.pos_enu = np.array([p.x, p.y, p.z])

    def _on_velocity(self, msg):
        v = msg.twist.linear
        self.airspeed = float(np.linalg.norm([v.x, v.y, v.z]))

    def _on_speed_scale(self, msg):
        self.wind_speed_scale = float(msg.data)

    def _tick(self):
        steady = cap_speed(self.grid.at(self.pos_enu) * self.wind_speed_scale,
                           self.mc_max_wind_speed_ms)
        wind_enu = steady + self.gust.step(self.airspeed)

        msg = Wind()
        msg.linear_velocity.x = float(wind_enu[0])
        msg.linear_velocity.y = float(wind_enu[1])
        msg.linear_velocity.z = float(wind_enu[2])
        msg.enable_wind = True
        self.gz_pub.publish(msg)

        wind_ned = frames.enu_to_ned(wind_enu)
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'ned'
        out.vector.x, out.vector.y, out.vector.z = (float(v) for v in wind_ned)
        self.truth_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = WindFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
