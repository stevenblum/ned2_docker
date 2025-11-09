#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # --- Paths to the packages ---
    niryo_driver_pkg = get_package_share_directory('niryo_ned_ros2_driver')
    moveit_pkg = get_package_share_directory('niryo_ned2_moveit_config')

    # --- Include the Niryo NED2 driver launch ---
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(niryo_driver_pkg, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'drivers_list_file': '/scripts/drivers_list_sim.yaml',
            'use_sim_time': 'false',
        }.items(),
    )

    # --- Include the MoveIt launch ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'ned2_moveit_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items(),
    )

    

    # --- Launch both together ---
    return LaunchDescription([
        driver_launch,
        moveit_launch,
    ])
