#!/usr/bin/python3
# -*- coding: utf-8 -*-

''' 
*****************************************************************************************
*
*        =============================================
*                  CL Theme (eYRC 2023-24)
*        =============================================
*
*
*  Filename:			ebot_bringup_launch.py
*  Description:         Use this file to start navigation on pre-generated map.
*  Created:				16/07/2023
*  Last Modified:	    10/09/2023
*  Modified by:         Ravikumar
*  Author:				e-Yantra Team
*  
*****************************************************************************************
'''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    opencv_dir = get_package_share_directory('ur_description')
    moveit_dir = get_package_share_directory('ur5_moveit')
    launch_dir = os.path.join(bringup_dir, 'launch')
    ebot_nav2_dir = get_package_share_directory('ebot_nav2')
    py_moveit_dir = get_package_share_directory('pymoveit2')


    pymoveit2_node = Node(
        package='pymoveit2',
        executable='task3b.py',
        name="task3b_moveit",
        output="screen"
    )

    ebot_nav_cmd = Node(
        package='ebot_nav2',
        executable='ebot_nav_cmd.py',
        name='ebot_nav',
        output='screen'
    )



    ld = LaunchDescription()
    ld.add_action(ebot_nav_cmd)
    ld.add_action(pymoveit2_node)


    return ld