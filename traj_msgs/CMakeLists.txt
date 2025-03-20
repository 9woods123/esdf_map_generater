cmake_minimum_required(VERSION 2.8.3)
project(traj_msgs)

find_package(catkin REQUIRED COMPONENTS 
  std_msgs
  geometry_msgs 
  message_generation)

add_message_files(
  FILES
  Trajectory.msg
  TrajectoryPoint.msg
 )

 ## Generate services in the 'srv' folder

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs 
    geometry_msgs 
    message_runtime
)
