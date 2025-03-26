#!/usr/bin/env python3
#
# Copyright 2025 NTREX CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    package_share_dir = get_package_share_directory('stella_bringup')
    param_file_path = os.path.join(package_share_dir, 'param', 'robot_launch_param.yaml')
    
    # Get parameter from YAML
    with open(param_file_path, 'r') as f:
        param = yaml.safe_load(f)

    launch_lidar2 = param.get('launch_lidar2', True)
    launch_usb_cam = param.get('launch_usb_cam', False)
    launch_realsense = param.get('launch_realsense', False)
    launch_pointcloud = param.get('launch_pointcloud', False)
    launch_pointcloud_laserscan_filter = param.get('launch_pointcloud_laserscan_filter', False)
    launch_hailo = param.get('launch_hailo', False)

    md_pkg_dir = LaunchConfiguration(
        'md_pkg_dir',
        default=os.path.join(get_package_share_directory('stella_md'), 'launch'))

    ahrs_pkg_dir = LaunchConfiguration(
        'ahrs_pkg_dir',
        default=os.path.join(get_package_share_directory('stella_ahrs'), 'launch'))

    lidar_pkg_dir = LaunchConfiguration(
        'lidar_pkg_dir',
        default=os.path.join(get_package_share_directory('sllidar_ros2'), 'launch'))

    lidar2_pkg_dir = LaunchConfiguration(
        'lidar2_pkg_dir',
        default=os.path.join(get_package_share_directory('sllidar2_ros2'), 'launch'))

    depth_pkg_dir = LaunchConfiguration(
        'depth_pkg_dir',
        default=os.path.join(get_package_share_directory('realsense2_camera'), 'launch'))

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='Use simulation (Gazebo) clock if true'),
        
        # Default state publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/stella_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition('false' if (launch_usb_cam or launch_realsense) else 'true')
        ),

        # state publisher with realsense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/stella_state_publisher_realsense.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition('true' if launch_realsense else 'false')
        ),

        # state publisher with realsense
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [ThisLaunchFileDir(), '/stella_state_publisher_web_cam.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
            condition=IfCondition('true' if launch_usb_cam else 'false')
        ),

        # MotorDriver launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([md_pkg_dir, '/stella_md_launch.py'])
        ),

        # AHRS launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ahrs_pkg_dir, '/stella_ahrs_launch.py'])
        ),

        # Upper lidar launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar_pkg_dir, '/sllidar_c1_launch.py'])
        ),

        # Bottom lidar launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar2_pkg_dir, '/sllidar_c1_launch.py']),
            condition=IfCondition('true' if launch_lidar2 else 'false')
        ),

        # Bottom lidar filter launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([lidar2_pkg_dir, '/laser_filter_launch.py']),
            condition=IfCondition('true' if launch_lidar2 else 'false')
        ),

        # Default realsense launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([depth_pkg_dir, '/rs_launch.py']),
            condition=IfCondition('true' if (launch_realsense and not launch_pointcloud and not launch_hailo) else 'false')
        ),

        # Pointcloud realsense launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([depth_pkg_dir, '/pc_rs_launch.py']),
            condition=IfCondition('true' if (launch_realsense and launch_pointcloud and not launch_hailo) else 'false')
        ),

        # Hailo realsense launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([depth_pkg_dir, '/hailo_rs_launch.py']),
            condition=IfCondition('true' if (launch_realsense and not launch_pointcloud and launch_hailo) else 'false')
        ),

        # v4l2(usb camera) node
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            condition=IfCondition('true' if launch_usb_cam else 'false')
        ),

        # Pointcloud pcl filter node
        Node(
            package='stella_pointcloud_handler',
            executable="pointcloud_passthrough_filter",
            name="pointcloud_passthrough_filter",
            output='screen',
            condition=IfCondition('true' if (launch_realsense and launch_pointcloud and not launch_hailo) else 'false')
        ),

        # Pointcloud to laserscan node
        Node(
            package='stella_pointcloud_handler',
            executable="pointcloud_to_laserscan_tf_node",
            name="pointcloud_to_laserscan_tf_node",
            output='screen',
            condition=IfCondition('true' if (launch_realsense and launch_pointcloud and launch_pointcloud_laserscan_filter) else 'false')
        ),

        # hailo rpi ros2 example node for usb camera
        Node(
            package='stella_hailo_rpi5_ros2_examples',
            executable='hailo_ros2_detection_node',
            name='hailo_ros2_detection_node',
            output='screen',
            condition=IfCondition('true' if (launch_usb_cam and launch_hailo) else 'false')
        ),

        # hailo rpi ros2 example node for realsense
        Node(
            package='stella_hailo_rpi5_ros2_examples',
            executable='hailo_ros2_detection_node',
            name='hailo_ros2_detection_node',
            remappings=[
                ('image_raw', '/camera/camera/color/image_raw')
            ],
            output='screen',
            condition=IfCondition('true' if (launch_realsense and not launch_pointcloud and launch_hailo) else 'false')
        ),

    ])

