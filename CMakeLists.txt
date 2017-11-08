cmake_minimum_required(VERSION 2.8.3)
project(traversal_map)

## Add support for C++11, supported in ROS Kinetic and newer
 add_definitions(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dynamic_reconfigure
  eigen_conversions
  geometry_msgs
  grid_map_msgs
  nav_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
find_package(octomap REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV)

include_directories(${PCL_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${OCTOMAP_LIBRARIES})
add_definitions(${PCL_DEFINITIONS})

generate_dynamic_reconfigure_options(
  cfg/TraversalArea.cfg
)

catkin_package(
 INCLUDE_DIRS
  LIBRARIES OpenCV
  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs std_msgs tf costmap_2d dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
add_library(teraversal_map_node src/traversal_map.cpp)

add_executable(${PROJECT_NAME}_node src/traversal_map.cpp
     include/traversal_map/traversal_map.h )

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)



target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )
