"""Run the trained FO-PINN at CONTROL_HZ and publish the wind estimate.

Publishes the estimated wind FORCE in NED newtons. The controller turns
that into a feedforward acceleration by dividing by mass.
"""
import pathlib

import numpy as np
import rclpy
import torch
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import SensorCombined, VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from . import constants as C
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
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, qos_profile_sensor_data)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._on_attitude, qos_profile_sensor_data)
        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self._on_imu, qos_profile_sensor_data)

        self.pub = self.create_publisher(Vector3Stamped, C.TOPIC_WIND_EST, 10)
        self.create_timer(1.0 / C.CONTROL_HZ, self._tick)

    def _on_position(self, m):
        self.vel = np.array([m.vx, m.vy, m.vz])

    def _on_attitude(self, m):
        self.quat = np.array([m.q[0], m.q[1], m.q[2], m.q[3]])

    def _on_imu(self, m):
        self.gyro = np.array(m.gyro_rad[:3])
        self.accel = np.array(m.accelerometer_m_s2[:3])

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
