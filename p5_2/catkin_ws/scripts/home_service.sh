#!/bin/sh
CATKIN_WS=/home/ubuntu/robotics_udacity/p5_2/catkin_ws
export TURTLEBOT_3D_SENSOR=kinect
export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib
export TURTLEBOT_GAZEBO_WORLD_FILE=$CATKIN_WS/src/my_robot/worlds/office.world
export TURTLEBOT_GAZEBO_MAP_FILE=$CATKIN_WS/src/my_robot/maps/map.yaml
export ROBOT_INITIAL_POSE="-x 0 -y 0 -z 0.0"

xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export GAZEBO_PLUGIN_PATH=$CATKIN_WS/devel/lib:/opt/ros/noetic/lib; export TURTLEBOT_GAZEBO_WORLD_FILE=$CATKIN_WS/src/my_robot/worlds/office.world; export ROBOT_INITIAL_POSE='-x 0 -y 0 -z 0.0'; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5

xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export TURTLEBOT_GAZEBO_MAP_FILE=$CATKIN_WS/src/my_robot/maps/map.yaml; roslaunch turtlebot_gazebo amcl_demo.launch initial_pose_x:=0 initial_pose_y:=0 initial_pose_a:=0" &
sleep 5

xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; rosrun rviz rviz -d $CATKIN_WS/src/add_markers/rviz/home_service.rviz" &
sleep 5

source /opt/ros/noetic/setup.bash
source $CATKIN_WS/devel/setup.bash
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 0, y: 0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.06853892326654787]}}'
sleep 2

xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; rosrun pick_objects pick_objects" &
sleep 2

xterm -hold -e "source /opt/ros/noetic/setup.bash; source $CATKIN_WS/devel/setup.bash; rosrun add_markers add_markers_service" &
