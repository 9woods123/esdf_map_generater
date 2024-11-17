cmake_minimum_required(VERSION 3.0.2)
project(esdf_map_generator)

find_package(catkin REQUIRED COMPONENTS
  mavros
  mavros_msgs
  rospy
  std_msgs
  roscpp
  octomap_msgs
  
)

find_package(octomap REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES fmt_ros
 CATKIN_DEPENDS mavros mavros_msgs rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${octomap_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

add_library(esdf_map_generator src/esdf_map_generator/esdf_map_generator.cpp)

target_link_libraries(esdf_map_generator
  ${catkin_LIBRARIES}
  ${octomap_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(esdf_map_generator_node src/esdf_map_generator/esdf_map_generator_node.cpp)

target_link_libraries(esdf_map_generator_node
esdf_map_generator
)