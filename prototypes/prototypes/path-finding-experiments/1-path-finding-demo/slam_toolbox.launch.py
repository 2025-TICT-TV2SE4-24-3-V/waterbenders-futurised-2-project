"""
De onderstaande code had ik niet kunnen realiseren zonder het prototype van Radeiaan voor de Lidar. Vandaar ook dat de code bijna identiek is.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': '/workspace/prototypes/path-finding-demo/slam_toolbox.yaml'
        }]
    )

    # Fixed coordinate relationship that tells ROS where the Lidar sensor is located relative to the robots body
    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=[
            '0.6', '0', '0.3', '0', '0', '0',
            'chassis', 'laser'
        ],
        output='screen'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': '/workspace/prototypes/path-finding-demo/slam_params.yaml',
            'use_sim_time': 'true'
        }.items()
    )

    map_listener = Node(
        package='map_listener',
        executable='map_subscriber',
        name='map_subscriber',
        output='screen'
    )

    return LaunchDescription([
        bridge,
        static_laser_tf,
        slam
        # map_listener
    ])
