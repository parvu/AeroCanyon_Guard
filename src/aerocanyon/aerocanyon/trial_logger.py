"""Log everything a trial needs, to one CSV per run.

This CSV is both the PINN's training set and the figures' input, so it
carries ground-truth wind alongside the state. Keep the column order
stable -- train_pinn.py and plot_results.py both read it by name, but a
human comparing two runs reads it by eye.
"""
import csv
import pathlib

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from px4_msgs.msg import (SensorCombined, VehicleAttitude, VehicleLocalPosition)
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from . import constants as C

COLUMNS = [
    't',
    'x', 'y', 'z', 'vx', 'vy', 'vz',
    'qw', 'qx', 'qy', 'qz',
    'ax', 'ay', 'az', 'p', 'q', 'r',
    'wind_true_n', 'wind_true_e', 'wind_true_d',
    'wind_est_n', 'wind_est_e', 'wind_est_d',
    'sp_n', 'sp_e', 'sp_d',
    'cbf_active', 'cbf_h_obstacle',
]


class TrialLogger(Node):

    def __init__(self):
        super().__init__('trial_logger')
        self.declare_parameter('trial', 'trial')
        self.declare_parameter('mode', 'baseline')
        self.declare_parameter('out_dir', 'trials')

        out = pathlib.Path(self.get_parameter('out_dir').value)
        out.mkdir(parents=True, exist_ok=True)
        name = (f"{self.get_parameter('trial').value}_"
                f"{self.get_parameter('mode').value}.csv")
        self.path = out / name
        self.fh = self.path.open('w', newline='')
        self.writer = csv.DictWriter(self.fh, fieldnames=COLUMNS)
        self.writer.writeheader()
        self.get_logger().info(f'logging to {self.path}')

        self.row = {c: 0.0 for c in COLUMNS}
        self.t0 = self.get_clock().now()

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._on_position, qos_profile_sensor_data)
        self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._on_attitude, qos_profile_sensor_data)
        self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined',
            self._on_imu, qos_profile_sensor_data)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_TRUTH, self._on_truth, 10)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_WIND_EST, self._on_est, 10)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_SETPOINT_DESIRED, self._on_sp, 10)
        self.create_subscription(
            Vector3Stamped, C.TOPIC_CBF_DIAG, self._on_cbf, 10)

        self.create_timer(1.0 / C.CONTROL_HZ, self._write)

    def _on_position(self, m):
        self.row.update(x=m.x, y=m.y, z=m.z, vx=m.vx, vy=m.vy, vz=m.vz)

    def _on_attitude(self, m):
        self.row.update(qw=m.q[0], qx=m.q[1], qy=m.q[2], qz=m.q[3])

    def _on_imu(self, m):
        self.row.update(ax=m.accelerometer_m_s2[0], ay=m.accelerometer_m_s2[1],
                        az=m.accelerometer_m_s2[2],
                        p=m.gyro_rad[0], q=m.gyro_rad[1], r=m.gyro_rad[2])

    def _on_truth(self, m):
        self.row.update(wind_true_n=m.vector.x, wind_true_e=m.vector.y,
                        wind_true_d=m.vector.z)

    def _on_est(self, m):
        self.row.update(wind_est_n=m.vector.x, wind_est_e=m.vector.y,
                        wind_est_d=m.vector.z)

    def _on_sp(self, m):
        self.row.update(sp_n=m.vector.x, sp_e=m.vector.y, sp_d=m.vector.z)

    def _on_cbf(self, m):
        # x = 1.0 if the filter modified the command, y = obstacle barrier
        # value (metres) -- never the stall barrier, see cbf_filter.py.
        self.row.update(cbf_active=m.vector.x, cbf_h_obstacle=m.vector.y)

    def _write(self):
        self.row['t'] = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.writer.writerow(self.row)
        self.fh.flush()

    def destroy_node(self):
        self.fh.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrialLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
