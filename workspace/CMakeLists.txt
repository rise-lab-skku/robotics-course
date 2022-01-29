cmake_minimum_required(VERSION 3.0.2)
project(workspace)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  moveit_core
  moveit_ros_planning_interface
  moveit_visual_tools
  rviz_visual_tools
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    ${THIS_PACKAGE_INCLUDE_DIRS}
  LIBRARIES
  CATKIN_DEPENDS
    trajectory_msgs
    moveit_core
    moveit_ros_planning_interface
    moveit_visual_tools
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simple_drawing src/simple_drawing.cpp)
target_link_libraries(simple_drawing
  ${catkin_LIBRARIES}
)

add_executable(reachable_ws_fk src/reachable_ws_fk.cpp)
target_link_libraries(reachable_ws_fk
  ${catkin_LIBRARIES}
)

add_executable(reachable_ws_ik src/reachable_ws_ik.cpp)
target_compile_options(reachable_ws_ik PRIVATE -O3)
target_link_libraries(reachable_ws_ik
  ${catkin_LIBRARIES}
)
