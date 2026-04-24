#!/bin/bash
export TURTLEBOT_3D_SENSOR=kinect
export GAZEBO_PLUGIN_PATH=/home/ubuntu/robotics_udacity/p5/catkin_ws/devel/lib:/opt/ros/noetic/lib
export TURTLEBOT_GAZEBO_WORLD_FILE=/home/ubuntu/robotics_udacity/p5/catkin_ws/src/my_robot/worlds/office.world
export TURTLEBOT_GAZEBO_MAP_FILE=/home/ubuntu/robotics_udacity/p5/catkin_ws/src/my_robot/maps/map.yaml
export ROBOT_INITIAL_POSE="-x -4 -y -4 -z 0.0"
xterm -hold -e "source /opt/ros/noetic/setup.bash; source /home/ubuntu/robotics_udacity/p5/catkin_ws/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export GAZEBO_PLUGIN_PATH=/home/ubuntu/robotics_udacity/p5/catkin_ws/devel/lib:/opt/ros/noetic/lib; export TURTLEBOT_GAZEBO_WORLD_FILE=/home/ubuntu/robotics_udacity/p5/catkin_ws/src/my_robot/worlds/office.world; export ROBOT_INITIAL_POSE='-x -4 -y -4 -z 0.0'; roslaunch turtlebot_gazebo turtlebot_world.launch" &
sleep 5
xterm -hold -e "source /opt/ros/noetic/setup.bash; source /home/ubuntu/robotics_udacity/p5/catkin_ws/devel/setup.bash; export TURTLEBOT_3D_SENSOR=kinect; export TURTLEBOT_GAZEBO_MAP_FILE=/home/ubuntu/robotics_udacity/p5/catkin_ws/src/my_robot/maps/map.yaml; roslaunch turtlebot_gazebo amcl_demo.launch initial_pose_x:=-3.993 initial_pose_y:=3.802 initial_pose_a:=-1.5708" &
sleep 5
xterm -hold -e "source /opt/ros/noetic/setup.bash; source /home/ubuntu/robotics_udacity/p5/catkin_ws/devel/setup.bash; rosrun rviz rviz -d /home/ubuntu/robotics_udacity/p5/catkin_ws/src/add_markers/rviz/home_service.rviz" &
sleep 5
source /opt/ros/noetic/setup.bash
source /home/ubuntu/robotics_udacity/p5/catkin_ws/devel/setup.bash
rostopic pub -1 /initialpose geometry_msgs/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: -4, y: 4, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: -0.7140286376412575, w: 0.7001164936124343}}, covariance: [0.25,0,0,0,0,0,0,0.25,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.06853892326654787]}}'
sleep 2
xterm -hold -e "source /opt/ros/noetic/setup.bash; source /home/ubuntu/robotics_udacity/p5/catkin_ws/devel/setup.bash; rosrun pick_objects pick_objects" &
sleep 2
xterm -hold -e "source /opt/ros/noetic/setup.bash; source /home/ubuntu/robotics_udacity/p5/catkin_ws/devel/setup.bash; rosrun add_markers add_markers_service" &
