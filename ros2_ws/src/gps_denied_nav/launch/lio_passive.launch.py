import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_dir = get_package_share_directory('gps_denied_nav')
    config_dir = os.path.join(pkg_dir, 'config')

    # 1. Livox Driver Launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('livox_ros2_avia'), 'launch', 'livox_lidar_msg_launch.py')
        )
    )

    # 2. FAST-LIO2 Launch (passive config: no map, no cloud publish, no PCD save)
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'avia_passive.yaml', 'rviz': 'false'}.items()
    )

    # 3. LIO Output Adapter (subscribes directly to /Odometry from FAST-LIO2)
    lio_output_adapter = Node(
        package='gps_denied_nav',
        executable='lio_output_adapter',
        name='lio_output_adapter',
        output='screen'
    )

    # 4. Safety Supervisor
    safety_supervisor = Node(
        package='gps_denied_nav',
        executable='safety_supervisor',
        name='safety_supervisor',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'safety_limits.yaml'),
            os.path.join(config_dir, 'resource_limits.yaml')
        ]
    )

    # NOTE: The following are INTENTIONALLY excluded from the passive launch:
    # - avia_fastlio_adapter: its output topics are not connected to FAST-LIO2 input
    # - keyframe_manager: requires /cloud_registered which is disabled in passive config
    # - map_backend: depends on keyframes which won't be generated
    # - px4_bridge: no flight controller during passive testing

    return LaunchDescription([
        livox_launch,
        fastlio_launch,
        lio_output_adapter,
        safety_supervisor
    ])
