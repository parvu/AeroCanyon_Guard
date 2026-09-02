"""Run the trained FO-PINN at CONTROL_HZ and publish the wind estimate.

Publishes the estimated wind FORCE in NED newtons. The controller turns
that into a feedforward acceleration by dividing by mass.
"""
import pathlib

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from . import constants as C
from . import frames
from .fo_pinn import STATE_DIM, FractionalMemory, WindEstimator


class FoPinnNode(Node):

    def __init__(self):
        super().__init__('fo_pinn_node')
        self.declare_parameter('model_path', '')
        self.declare_parameter('enabled', 'treatment')

        # In baseline mode the node stays up but publishes nothing, so both
        # trials run the identical launch graph.
        self.enabled = self.get_parameter('enabled').value == 'treatment'

        path = self.get_parameter('model_path').value
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = str(pathlib.Path(get_package_share_directory('aerocanyon'))
                       / 'data' / 'wind_estimator.pt')

        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        ckpt = torch.load(path, map_location=self.device, weights_only=True)
        self.net = WindEstimator(hidden=ckpt['hidden']).to(self.device)
        self.net.load_state_dict(ckpt['state_dict'])
        self.net.eval()
        self.memory = FractionalMemory(alpha=ckpt['alpha'], n=ckpt['n'])
        self.get_logger().info(
            f"loaded {path} (alpha={ckpt['alpha']}, n={ckpt['n']}), "
            f"device={self.device}, enabled={self.enabled}")

        self.vel = np.zeros(3)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])
        self.gyro = np.zeros(3)
        self.accel = np.zeros(3)

        self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self._on_velocity, qos_profile_sensor_data)
        self.create_subscription(
            Imu, '/mavros/imu/data', self._on_imu, qos_profile_sensor_data)

        self.pub = self.create_publisher(Vector3Stamped, C.TOPIC_WIND_EST, 10)
        self.create_timer(1.0 / C.CONTROL_HZ, self._tick)

    def _on_velocity(self, m):
        v = m.twist.linear
        self.vel = frames.enu_to_ned([v.x, v.y, v.z])

    def _on_imu(self, m):
        q = m.orientation
        self.quat = frames.enu_flu_quat_to_ned_frd([q.w, q.x, q.y, q.z])
        g = m.angular_velocity
        self.gyro = frames.enu_flu_rate_to_ned_frd([g.x, g.y, g.z])
        a = m.linear_acceleration
        self.accel = frames.enu_flu_rate_to_ned_frd([a.x, a.y, a.z])

    def _tick(self):
        if not self.enabled:
            return
        state = np.concatenate([self.vel, self.quat, self.gyro, self.accel])
        assert state.shape == (STATE_DIM,), state.shape
        self.memory.push(state)
        x = torch.tensor(np.concatenate([state, self.memory.features()]),
                         dtype=torch.float32, device=self.device).unsqueeze(0)
        with torch.no_grad():
            f = self.net(x).squeeze(0).cpu().numpy()

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ned'
        msg.vector.x, msg.vector.y, msg.vector.z = (float(v) for v in f)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FoPinnNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
