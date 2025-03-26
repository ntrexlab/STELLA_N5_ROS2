from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            remappings=[('scan', 'scan_2')],# change sub topic scan -> scan_2
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("sllidar2_ros2"),
                    "filter_params", "laser_filter_param.yaml",
                ])],# 5~175 deg filter
        )
    ])
