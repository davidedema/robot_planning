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
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    shelfino_inflation = LaunchConfiguration('shelfino_inflation', default='0.0')  

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "shelfino_inflation",
                default_value="0.0",
                description="Inflation radius for Shelfino obstacles",
            ),
            
            Node(
                package="planner",
                executable="main_combinatorial",
                name="main_combinatorial",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, 'shelfino_inflation': shelfino_inflation},
                ],
            ),
            Node(
                package="planner",
                executable="nav2_client",
                name="nav2_client",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                ],
            ),
        ]
    )
