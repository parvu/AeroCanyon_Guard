"""Bring up the aerocanyon nodes. PX4 and the DDS agent are started
separately (see the repo README) -- ROS2 launch has no clean way to own
the PX4 shell, and trying to make it do so wastes more time than it saves.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode = LaunchConfiguration('mode')
    trial = LaunchConfiguration('trial')
    seed = LaunchConfiguration('seed')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='baseline',
                              description='baseline or treatment'),
        DeclareLaunchArgument('trial', default_value='trial',
                              description='CSV output basename'),
        DeclareLaunchArgument('seed', default_value='0',
                              description='Dryden gust RNG seed -- vary this '
                                          'across trials for wind diversity'),
        Node(package='aerocanyon', executable='wind_field_node',
             name='wind_field_node', output='screen',
             parameters=[{'seed': ParameterValue(seed, value_type=int)}]),
        Node(package='aerocanyon', executable='fo_pinn_node',
             name='fo_pinn_node', output='screen',
             parameters=[{'enabled': mode}]),
        Node(package='aerocanyon', executable='controller_node',
             name='controller_node', output='screen',
             parameters=[{'mode': mode}]),
        Node(package='aerocanyon', executable='trial_logger',
             name='trial_logger', output='screen',
             parameters=[{'trial': trial, 'mode': mode}]),
    ])
