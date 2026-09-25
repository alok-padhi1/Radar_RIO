import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('gps_denied_nav')
    config_dir = os.path.join(pkg_dir, 'config')

    # Livox Driver Launch
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('livox_ros2_avia'), 'launch', 'livox_lidar_msg_launch.py')
        )
    )

    # FAST-LIO2 Launch
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('fast_lio'), 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'config_file': 'avia.yaml'}.items()
    )

    # 1. Avia FAST-LIO Adapter
    avia_fastlio_adapter = Node(
        package='gps_denied_nav',
        executable='avia_fastlio_adapter',
        name='avia_fastlio_adapter',
        output='screen',
        parameters=[os.path.join(config_dir, 'avia_fastlio.yaml')]
    )

    # 2. LIO Output Adapter
    lio_output_adapter = Node(
        package='gps_denied_nav',
        executable='lio_output_adapter',
        name='lio_output_adapter',
        output='screen'
    )

    # 3. PX4 Bridge
    px4_bridge = Node(
        package='gps_denied_nav',
        executable='px4_bridge',
        name='px4_bridge',
        output='screen',
        parameters=[os.path.join(config_dir, 'px4_bridge.yaml')]
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

    # 5. Keyframe Manager
    keyframe_manager = Node(
        package='gps_denied_nav',
        executable='keyframe_manager',
        name='keyframe_manager',
        output='screen',
        parameters=[os.path.join(config_dir, 'keyframes.yaml')]
    )

    # 6. Map Backend
    map_backend = Node(
        package='gps_denied_nav',
        executable='map_backend',
        name='map_backend',
        output='screen',
        parameters=[os.path.join(config_dir, 'map_backend.yaml')]
    )

    return LaunchDescription([
        livox_launch,
        fastlio_launch,
        avia_fastlio_adapter,
        lio_output_adapter,
        px4_bridge,
        safety_supervisor,
        keyframe_manager,
        map_backend
    ])
