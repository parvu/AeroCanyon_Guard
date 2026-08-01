"""Pinn telemetry pipeline for the PX4/x500 sim: bridges the raw Gazebo IMU
topic into ROS2 and runs physics_bridge to publish /pinn/input_state.

Run this alongside the PX4 SITL stack (PX4-Autopilot's `make px4_sitl
gz_x500` with PX4_GZ_WORLD=dve_wind_arena, plus the Micro-XRCE-DDS-Agent) --
it doesn't launch Gazebo or PX4 itself, since those are owned by PX4's own
build/launch tooling, not this ROS2 workspace.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/dve_wind_arena/model/x500_0/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    physics_bridge_node = Node(
        package='phy_ai_simulation',
        executable='physics_bridge',
        output='screen'
    )

    return LaunchDescription([
        gz_bridge,
        physics_bridge_node,
    ])
