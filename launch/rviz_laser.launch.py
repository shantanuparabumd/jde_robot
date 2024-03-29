#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    launch_file_dir = get_package_share_directory('jde_robot')

 

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'launch', 'competition_world.launch.py')
        )
    )
       
    rviz_config_file = os.path.join(launch_file_dir, 'rviz', 'laser.rviz')
    
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # Specify the RViz configuration file
        )
    
    
    
    
    # Define delays between each launch
    timer1 = TimerAction(
        period=1.0,
        actions=[gazebo]
    )
    timer2 = TimerAction(
        period=5.0,
        actions=[rviz]
    )
   
    
    
    

    # Combine launch descriptions and timers
    return LaunchDescription([
        timer1, 
        timer2
    ])
