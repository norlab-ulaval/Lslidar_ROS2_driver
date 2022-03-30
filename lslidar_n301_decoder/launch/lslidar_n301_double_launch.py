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
    decoder_dir = os.path.join(get_package_share_directory('lslidar_n301_decoder'), 'params', 'lslidar_n301.yaml')
    decoder_dir1 = os.path.join(get_package_share_directory('lslidar_n301_decoder'), 'params', 'lslidar_n301_1.yaml')
    
    decoder_node = LifecycleNode(package='lslidar_n301_decoder',
                                executable='lslidar_n301_decoder_node',
                                name='lslidar_n301_decoder_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir],
                                )
                                
    driver_node = LifecycleNode(package='lslidar_n301_driver',
                                executable='lslidar_n301_driver_node',
                                name='lslidar_n301_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir],
                                )
                                
    decoder_node1 = LifecycleNode(package='lslidar_n301_decoder',
                                executable='lslidar_n301_decoder_node',
                                name='lslidar_n301_decoder_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[decoder_dir1],
                                )
                                
    driver_node1 = LifecycleNode(package='lslidar_n301_driver',
                                executable='lslidar_n301_driver_node',
                                name='lslidar_n301_driver_node',
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

