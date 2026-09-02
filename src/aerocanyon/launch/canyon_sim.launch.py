"""Bring up the aerocanyon nodes. Gazebo, ArduPilot SITL, and MAVROS are
started separately (see the repo README / run_trial.py) -- ROS2 launch
has no clean way to own those processes, and trying to make it do so
wastes more time than it saves.
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
        # Real bug found live: every node here uses create_timer()/get_clock()
        # with no use_sim_time set, and nothing bridged Gazebo's own /clock
        # into ROS2 -- so all of them ran on wall-clock ROS time while
        # Gazebo's <real_time_factor>1.0</real_time_factor> is only a TARGET,
        # not a guarantee. Under load (a concurrent colcon build, other gz
        # sim instances, etc.) the achieved real-time factor drifts, so a
        # fixed --seed still lands each Dryden gust sample (canyon_field.py's
        # DrydenGust.step(), one rng.normal() draw per wall-clock tick) at a
        # DIFFERENT point in simulated flight each run -- confirmed as the
        # root cause of four "identical seed" trials producing RMS-deviation
        # reductions of 0.1%, -104%, 14.2%, -5.4%. Bridging Gazebo's sim
        # clock and setting use_sim_time on every node locks all four timers
        # (wind, PINN, controller, logger) to simulated time instead.
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             parameters=[{'use_sim_time': True}]),
        Node(package='aerocanyon', executable='wind_field_node',
             name='wind_field_node', output='screen',
             parameters=[{'seed': ParameterValue(seed, value_type=int),
                          'turbulence_sigma': ParameterValue(turbulence, value_type=float),
                          'use_sim_time': True}]),
        Node(package='aerocanyon', executable='fo_pinn_node',
             name='fo_pinn_node', output='screen',
             parameters=[{'enabled': mode, 'use_sim_time': True}]),
        Node(package='aerocanyon', executable='controller_node',
             name='controller_node', output='screen',
             parameters=[{'mode': mode,
                          'feedforward_gain': ParameterValue(ff_gain, value_type=float),
                          'use_sim_time': True}]),
        Node(package='aerocanyon', executable='trial_logger',
             name='trial_logger', output='screen',
             parameters=[{'trial': trial, 'mode': mode, 'use_sim_time': True}]),
    ])
