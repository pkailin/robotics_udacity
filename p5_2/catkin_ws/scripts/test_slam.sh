#!/bin/sh
CATKIN_WS=/home/ubuntu/robotics_udacity/p5_2/catkin_ws
export TURTLEBOT_3D_SENSOR=kinect
export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib
export TURTLEBOT_GAZEBO_WORLD_FILE=$CATKIN_WS/src/my_robot/worlds/office.world
export ROBOT_INITIAL_POSE="-x -4 -y -4 -z 0.0"
xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib; export TURTLEBOT_GAZEBO_WORLD_FILE=$CATKIN_WS/src/my_robot/worlds/office.world; export ROBOT_INITIAL_POSE='-x -4 -y -4 -z 0.0'; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib; roslaunch turtlebot_gazebo gmapping_demo.launch" &
sleep 5
xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib; roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib; roslaunch turtlebot_teleop keyboard_teleop.launch" &
