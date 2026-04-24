cmake_minimum_required(VERSION 3.0.2)
project(add_markers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  visualization_msgs
  geometry_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(add_markers src/add_markers.cpp)
target_link_libraries(add_markers ${catkin_LIBRARIES})

add_executable(add_markers_service src/add_markers_service.cpp)
target_link_libraries(add_markers_service ${catkin_LIBRARIES})
