# Scene Setting Background Prompt
I am using Niryo's ned_ros ROS1 workspace to run a simulation 
of the ned2 robotic arm in gazebo classic in a docker container. I then have 
a seperate docker container that is running ned_ros2_driver 
ROS2 workspace. The driver container also runs moveit and some other custom nodes
to control the ned2 robot in the simulation container. 
I startup both containers with a docker compose file that also executes several 
basic scripts to starup gazebo, moveit, rviz, and several custom notes.
I can sucessfully control the simulated robot from the driver container with moveit. 