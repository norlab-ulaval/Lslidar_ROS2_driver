#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import lifecycle_msgs.msg
import os

def generate_launch_description():
    share_dir = get_package_share_directory('lsm10_v2')
    parameter_file = LaunchConfiguration('params_file')
    node_name = 'lsm10_node'

    params_declare = DeclareLaunchArgument('params_file',
                                           default_value=os.path.join(
                                           share_dir, 'params', 'lsm10_v2.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = LifecycleNode(package='lsm10_v2',
                                node_executable='lsm10_node',
                                node_name='lsm10_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                node_namespace='/',
                                )
         
    return LaunchDescription([
        params_declare,
        driver_node,
    ])

