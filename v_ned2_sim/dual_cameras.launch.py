from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "ros2", "run", "gazebo_ros", "spawn_entity.py",
                "-entity", "dual_cameras",
                "-file", "/root/scripts/dual_cameras.urdf",
                "-x", "0", "-y", "0", "-z", "0.5"
            ],
            output="screen"
        )
    ])
