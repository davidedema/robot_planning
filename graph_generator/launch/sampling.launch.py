#!/usr/bin/env python3
#
# Author: Placido Falqueto placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import OpaqueFunction

def print_env(context):
    print(__file__)
    for key in context.launch_configurations.keys():
        print("\t", key, context.launch_configurations[key])
    return

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='graph_generator',
            executable='main_sampling',
            name='main_sampling',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},],
        ),
        
        Node(
            package='graph_generator',
            executable='nav2_client',
            name='nav2_client',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},],
        )
    ])
