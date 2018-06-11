# RoboND-Home-Service-Robot
Final project of the Udacity Robotics Nanodegree in which a small home service robot is simulated in a Gazebo environment that it must first map by following the walls before then taking instructions to navigate to a target location and then to a second location following success on the first target. This simulates autonomous pick-up and drop-off of an item in the environment.

The project was undertaken on an NVIDIA Jetson TX2. A summary of the tasks include:

1. Design of a simple environment with the Building Editor in Gazebo.
2. Teleoperation of the robot and manual testing SLAM.
3. Creation of a wall_follower node that autonomously drives the robot to map the environment.
4. Use of the ROS navigation stack and which manually commands the robot using the 2D Nav Goal arrow in rviz to move to 2 different desired positions and orientations.
5. Creation of a pick_objects node that commands the robot to move to the desired pickup and drop off zones.
6. Creation of an add_markers node that subscribes to the robot odometry, keeps track of the robot pose, and publishes markers to rviz.

To use the project follow these steps:
+ Install ROS-Kinetic and Gazebo
+ First create a catkin workspace (e.g. $ mkdir ~/catkin_ws)
+ Clone the repo including the required submodules (e.g. turtlebot) use the following terminal command:
*git clone --recurse-submodules https://github.com/michaelhetherington/RoboND-Home-Service-Robot.git ~/catkin_ws/*
+ Build the workspace with *catkin_make* from the ~/catkin_ws/ directory.
+ At this stage there may be build errors that require installation of dependencies for submodules - use the reported errors to debug the issues and install dependencies as required
+ Run the project with the terminal command *.~/catkin_ws/src/ShellScripts/home_service.sh*
