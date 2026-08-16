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
    turbulence = LaunchConfiguration('turbulence_sigma')
    ff_gain = LaunchConfiguration('feedforward_gain')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='baseline',
                              description='baseline or treatment'),
        DeclareLaunchArgument('trial', default_value='trial',
                              description='CSV output basename'),
        DeclareLaunchArgument('seed', default_value='0',
                              description='Dryden gust RNG seed -- vary this '
                                          'across trials for wind diversity'),
        DeclareLaunchArgument('turbulence_sigma', default_value='2.5',
                              description='Dryden gust intensity, m/s. Raised '
                                          'from the original 1.5 -- see '
                                          'wind_field_node for why'),
        DeclareLaunchArgument('feedforward_gain', default_value='0.2',
                              description='scales the PINN feedforward; see '
                                          'controller_node'),
        Node(package='aerocanyon', executable='wind_field_node',
             name='wind_field_node', output='screen',
             parameters=[{'seed': ParameterValue(seed, value_type=int),
                          'turbulence_sigma': ParameterValue(turbulence, value_type=float)}]),
        Node(package='aerocanyon', executable='fo_pinn_node',
             name='fo_pinn_node', output='screen',
             parameters=[{'enabled': mode}]),
        Node(package='aerocanyon', executable='controller_node',
             name='controller_node', output='screen',
             parameters=[{'mode': mode,
                          'feedforward_gain': ParameterValue(ff_gain, value_type=float)}]),
        Node(package='aerocanyon', executable='trial_logger',
             name='trial_logger', output='screen',
             parameters=[{'trial': trial, 'mode': mode}]),
    ])
