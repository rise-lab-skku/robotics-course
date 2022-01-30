cmake_minimum_required(VERSION 3.0.2)
project(lidar_roi_example)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs 

)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_roi_example src/vlp16_roi.cpp src/main.cpp)
add_dependencies(lidar_roi_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(
  lidar_roi_example ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )

