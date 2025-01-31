#!/usr/bin/env python3
#
# Author: Placido Falqueto placido.falqueto [at] unitn.it
# Maintainer: Enrico Saccon  enrico.saccon [at] unitn.it

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    shelfino_inflation = LaunchConfiguration('shelfino_inflation', default='0.5')  

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),

        DeclareLaunchArgument(  
            'shelfino_inflation',
            default_value='0.5',
            description='Inflation radius for Shelfino obstacles'
        ),

        Node(
            package='planner',
            executable='main_sampling',
            name='main_sampling',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'shelfino_inflation': shelfino_inflation}],
        ),
        
        Node(
            package='planner',
            executable='nav2_client',
            name='nav2_client',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],  # This node may not need shelfino_inflation
        )
    ])
