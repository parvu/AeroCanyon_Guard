import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('phy_ai_simulation')
    world_path = os.path.join(pkg_share, 'worlds', 'dve_wind_arena.sdf')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # IMU: Gazebo -> ROS  ([  means Gz->ROS)
            '/world/dve_wind_arena/model/pinn_drone/link/base_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # cmd_vel: ROS -> Gazebo  (]  means ROS->Gz)
            '/pinn_drone/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        ],
        output='screen'
    )

    physics_bridge_node = Node(
        package='phy_ai_simulation',
        executable='physics_bridge',
        output='screen'
    )

    # NOTE: live_inference_node is disabled for keyboard control.
    # The PINN model has random (untrained) weights and publishes at 250 Hz,
    # drowning out keyboard commands. Re-enable once the model is trained.
    # live_inference_node = Node(
    #     package='phy_ai_simulation',
    #     executable='live_inference_node.py',
    #     output='screen'
    # )

    # MulticopterVelocityControl starts disabled and only listens on Gazebo
    # transport (not ROS), so it can't be reached via ros_gz_bridge. Enable it
    # directly once the world has had time to load.
    enable_motors = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['gz', 'topic', '-t', '/pinn_drone/enable',
                 '-m', 'gz.msgs.Boolean', '-p', 'data: true'],
            output='screen'
        )]
    )

    return LaunchDescription([
        gz_sim,
        gz_bridge,
        physics_bridge_node,
        enable_motors,
        # live_inference_node,   # disabled: untrained PINN overrides keyboard
    ])

