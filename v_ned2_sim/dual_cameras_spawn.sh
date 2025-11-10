#!/bin/bash

# rosservice call /gazebo/delete_model '{model_name: cam_right}'


source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

rosrun gazebo_ros spawn_model \
    -urdf \
    -file "/volume/single_camera.urdf" \
    -model "cam_left" \
    -x .5 -y "-0.5" -z .25 \
    -R 0.0 -P 0.3 -Y 2.2 \
    -robot_namespace "/cam_left"

rosrun gazebo_ros spawn_model \
    -urdf \
    -file "/volume/single_camera.urdf" \
    -model "cam_right" \
    -x .5 -y .5 -z .25 \
    -R 0.0 -P 0.4 -Y "-2.0" \
    -robot_namespace "/cam_right"



