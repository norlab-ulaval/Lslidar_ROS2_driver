#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os

def generate_launch_description():
    decoder_dir = os.path.join(get_package_share_directory('lidar_c32_decoder'), 'params', 'lidar_c32_1.yaml')
    decoder_dir1 = os.path.join(get_package_share_directory('lidar_c32_decoder'), 'params', 'lidar_c32.yaml')
    
    decoder_node = LifecycleNode(package='lidar_c32_decoder',
                                executable='cloud_node',
                                name='cloud_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir],
                                )
                                
    driver_node = LifecycleNode(package='lidar_c32_driver',
                                executable='lidar_node',
                                name='lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir],
                                )
    decoder_node1 = LifecycleNode(package='lidar_c32_decoder',
                                executable='cloud_node',
                                name='cloud_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir1],
                                )
                                
    driver_node1 = LifecycleNode(package='lidar_c32_driver',
                                executable='lidar_node',
                                name='lidar_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir1],
                                )
                                                            
    return LaunchDescription([
        decoder_node,
        driver_node,
        decoder_node1,
        driver_node1,
    ])

