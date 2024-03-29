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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
   


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'launch', 'competition_world.launch.py')
        )
    )
    map_location = os.path.join(get_package_share_directory('jde_robot'), 'maps', 'final_map.yaml')
     
     
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={'map': map_location, 'use_sim_time': use_sim_time}.items()
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    
    
    
    rviz_config_file = os.path.join(launch_file_dir, 'rviz', 'final.rviz')
    
    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]  # Specify the RViz configuration file
        )
    
    init = Node(
            package='jde_robot',
            executable='init_pose.py',
            name='init_pose'
        )
    
    way_point = Node(
            package='jde_robot',
            executable='waypoint.py',
            name='way_point_nav'
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
    timer3 = TimerAction(
        period=10.0,
        actions=[localization]
    )
    timer4 = TimerAction(
        period=15.0,
        actions=[navigation]
    )
    timer5 = TimerAction(
        period=20.0,
        actions=[init]
    )
    
    timer6 = TimerAction(
        period=25.0,
        actions=[way_point]
    )
    
    
    
    
    

    # Combine launch descriptions and timers
    return LaunchDescription([
        timer1, 
        timer2, 
        timer3, 
        timer4,
        timer5,
        timer6
    ])
