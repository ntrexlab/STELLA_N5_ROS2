#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    
    driver_node = LifecycleNode(package='stella_md',
                                executable='stella_md_node',
                                name='stella_md_node',
                                output='screen',
                                emulate_tty=True,
                                namespace='/',
                                )

    
    return LaunchDescription([
       driver_node,
    ])