#!/bin/bash

# Test SLAM script - launches all required nodes for SLAM testing

# Launch turtlebot world in Gazebo
xterm -e "roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

# Launch gmapping SLAM
xterm -e "roslaunch slam_gmapping gmapping_demo.launch" &

sleep 3

# Launch rviz to visualize the map
xterm -e "roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 3

# Launch keyboard teleop to control the robot
xterm -e "roslaunch turtlebot_teleop keyboard_teleop.launch" &
