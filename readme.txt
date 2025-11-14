# Scene Setting Background Prompt
I am using Niryo's ned_ros ROS1 workspace to run a simulation 
of the ned2 robotic arm in gazebo classic in a docker container. I then have 
a seperate docker container that is running ned_ros2_driver 
ROS2 workspace. The driver container also runs moveit and some other custom nodes
to control the ned2 robot in the simulation container. 
I startup both containers with a docker compose file that also executes several 
basic scripts to starup gazebo, moveit, rviz, and several custom notes.
I can sucessfully control the simulated robot from the driver container with moveit. 

source /opt/ros/jazzy/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/ros2_drivers_ws/install/setup.bash
source ~/stacks_ws/install/setup.bash
source ~/ros2_driver_venv/bin/activate

ros2

#!/usr/bin/env python3
import sys
import argparse

import rclpy
from rclpy.logging import get_logger

from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy


# Default pose: position (x,y,z) and "gripper down" orientation
DEFAULT_POSITION = (0.2, 0.0, 0.3)
# 180 degrees about x-axis -> gripper z-axis down in a typical base_link frame
DEFAULT_ORIENTATION = (1.0, 0.0, 0.0, 0.0)  # (qx, qy, qz, qw)

GROUP_NAME = "arm"
POSE_LINK = "tool_link"  # change to your tip_link if different (e.g. "TCP")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Move NED2 arm to a target pose using MoveItPy.",
        add_help=True,
    )

    # Position
    parser.add_argument("--x", type=float, default=DEFAULT_POSITION[0],
                        help=f"Target x [m] (default: {DEFAULT_POSITION[0]})")
    parser.add_argument("--y", type=float, default=DEFAULT_POSITION[1],
                        help=f"Target y [m] (default: {DEFAULT_POSITION[1]})")
    parser.add_argument("--z", type=float, default=DEFAULT_POSITION[2],
                        help=f"Target z [m] (default: {DEFAULT_POSITION[2]})")

    # Orientation as quaternion (optional). If any is omitted, we use the default.
    parser.add_argument("--qx", type=float, help="Quaternion x (default: gripper down)")
    parser.add_argument("--qy", type=float, help="Quaternion y (default: gripper down)")
    parser.add_argument("--qz", type=float, help="Quaternion z (default: gripper down)")
    parser.add_argument("--qw", type=float, help="Quaternion w (default: gripper down)")

    # Separate our arguments from ROS arguments
    args, ros_args = parser.parse_known_args(argv)
    return args, ros_args


def plan_and_execute(robot, planning_component, logger):
    logger.info("Planning trajectory...")
    plan_result = planning_component.plan()
    if not plan_result:
        logger.error("Planning failed.")
        return False

    logger.info("Executing trajectory...")
    robot.execute(plan_result.trajectory, controllers=[])
    logger.info("Execution complete.")
    return True


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    cli_args, ros_args = parse_args(argv)
    logger = get_logger("ned2_move_to_pose_cli")

    # Use provided quaternion if all four components are given, otherwise default
    if all(v is not None for v in (cli_args.qx, cli_args.qy, cli_args.qz, cli_args.qw)):
        qx, qy, qz, qw = cli_args.qx, cli_args.qy, cli_args.qz, cli_args.qw
    else:
        qx, qy, qz, qw = DEFAULT_ORIENTATION

    x, y, z = cli_args.x, cli_args.y, cli_args.z

    logger.info(
        f"Target pose in base_link:\n"
        f"  position = ({x:.3f}, {y:.3f}, {z:.3f})\n"
        f"  orientation (qx,qy,qz,qw) = ({qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f})"
    )

    # Init ROS and MoveItPy
    rclpy.init(args=ros_args)
    robot = MoveItPy(node_name="ned2_moveit_py_cli")
    arm = robot.get_planning_component(GROUP_NAME)

    # Start from the current robot state
    arm.set_start_state_to_current_state()

    # Build pose goal
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.position.x = x
    pose_goal.pose.position.y = y
    pose_goal.pose.position.z = z
    pose_goal.pose.orientation.x = qx
    pose_goal.pose.orientation.y = qy
    pose_goal.pose.orientation.z = qz
    pose_goal.pose.orientation.w = qw

    # Set goal state for the EE link
    arm.set_goal_state(
        pose_stamped_msg=pose_goal,
        pose_link=POSE_LINK,
    )

    # Plan + execute
    success = plan_and_execute(robot, arm, logger)

    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
