cmake_minimum_required(VERSION 3.0.2)
project(traj_server)

# 查找依赖包
find_package(catkin REQUIRED COMPONENTS
  mavros_msgs
  roscpp
  std_msgs
  traj_msgs
)

find_package(Eigen3 REQUIRED)

# 定义 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# 配置 catkin 包
catkin_package(
  LIBRARIES traj_server
  CATKIN_DEPENDS mavros_msgs roscpp std_msgs
)

# 包含头文件目录
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# 添加可执行文件
add_executable(traj_server src/traj_server.cpp)

# 链接库
target_link_libraries(traj_server
  ${catkin_LIBRARIES}
  ${Eigen3_LIBRARIES}
)