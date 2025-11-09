#!/bin/bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
rosrun gazebo_ros spawn_model \
  -file /cameras/dual_cameras.urdf \
  -urdf -x 0 -y 0 -z 0.5 -model dual_cameras
