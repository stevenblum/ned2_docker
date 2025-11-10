#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction
from launch.actions import ExecuteProcess
from rclpy import logging

logger = logging.get_logger("ned2_moveit.launch")

def generate_launch_description():
    
    niryo_driver_pkg = get_package_share_directory("niryo_ned_ros2_driver")

    urdf_file = os.path.join(
        get_package_share_directory("niryo_ned_description"),
        "urdf/ned2",
        #"niryo_ned2.urdf.xacro",
        "niryo_ned2_gripper1_n_camera.urdf.xacro"

    )
    srdf_file = "/scripts/sim/niryo_ned2_gripper.srdf"

    path_clock_republisher = "/scripts/sim/republish_clock.py"

    path_driver_list_yaml = '/scripts/sim/drivers_list_sim.yaml'
    path_whitelist_params_yaml = '/scripts/sim/whitelist_sim.yaml'

    # -------------------------------------------------------------------------
    # Declare arguments
    # -------------------------------------------------------------------------
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    # -------------------------------------------------------------------------
    # Build MoveIt configuration
    # -------------------------------------------------------------------------

    moveit_config = (
        MoveItConfigsBuilder("niryo_ned2")
        .robot_description(file_path=urdf_file)
        .joint_limits(file_path="config/joint_limits.yaml")
        .robot_description_semantic(file_path=srdf_file)
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner", "stomp"]
        )
        .to_moveit_configs()
    )

    # -------------------------------------------------------------------------
    # Move Group Node
    # -------------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"collision_detection": {"allow_self_collision": False}},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    move_group_delayed = TimerAction(period=5.0, actions=[move_group_node])

    # -------------------------------------------------------------------------
    # RViz Node
    # -------------------------------------------------------------------------
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("niryo_ned2_moveit_config"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    ros2_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(niryo_driver_pkg, 'launch', 'driver.launch.py')
        ),
        launch_arguments={
            'drivers_list_file': path_driver_list_yaml,
            'whitelist_params_file': path_whitelist_params_yaml,
            'use_sim_time': 'true'
        }.items(),
    )

    # DONT NEED CLOCK REPUBLISHER, BECUASE I HAVE THE ROSBRIDGE NODE PUBLISHING CLOCK
    #clock_republisher = ExecuteProcess(
    #    cmd=["python3", path_clock_republisher],
    #    output="screen",
    #)

    # -------------------------------------------------------------------------
    # Launch Description
    # -------------------------------------------------------------------------
    return LaunchDescription([
        ros2_driver_launch,
        clock_republisher,
        rviz_config_arg,
        use_sim_time_arg,
        rviz_node,
        move_group_delayed,
    ])
