#include <ros/ros.h>
#include "esdf_map_generator/esdf_map_generator.h"

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "esdf_map_generator_node");
    ros::NodeHandle nh;

    // 创建 ESDFMapGenerator 对象
    ESDFMapGenerator esdf_map_generator;

    // 启动 ROS 循环
    ros::spin();

    return 0;
}
