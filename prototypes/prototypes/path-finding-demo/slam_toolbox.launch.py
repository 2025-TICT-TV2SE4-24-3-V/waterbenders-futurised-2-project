"""
Disclaimer:
De onderstaande code is volledig mogelijk gemaakt door Radeiaan zijn prototype voor de Lidar. Het is letterlijk geplakt en gekopieërd.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': '/workspace/prototypes/path-finding-demo/ros_bridges.yaml'
        }]
    )

    static_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_static_tf',
        arguments=[
            '0.6', '0', '0.3', '0', '0', '0',
            'chassis', 'FLIP/chassis/gpu_lidar'
        ],
        output='screen'
    )

    slam_launch = IncludeLaunchDescription(
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

    path_finding = Node(
        package='path_finding',
        executable='main',
        name='path_finding_node',
        output='screen',
    )

    return LaunchDescription([
        bridge,
        static_laser_tf,
        slam_launch,
        # path_finding # Comment line for easier debugging of 'path_finding/main.py'
    ])
