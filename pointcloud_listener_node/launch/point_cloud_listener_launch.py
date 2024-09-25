import os
from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package = get_package_share_directory("pointcloud_listener_node")
    config_path = os.path.join(package, "config", "config.yaml")

    return LaunchDescription([
        Node(
            package='pointcloud_listener_node',
            executable='pointcloud_listener_node',
            output='screen',
            parameters=[config_path]
        )
    ])
