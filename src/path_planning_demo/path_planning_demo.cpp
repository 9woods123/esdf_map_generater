#include "path_planner/path_planner.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <iostream>

// 全局变量
ros::Subscriber position_sub;
geometry_msgs::Point robot_position;  // 存储机器人位置
bool is_get_start_point = false;

// 目标位置
double goal_x, goal_y, goal_z;
bool is_2D_planning=false;
double fixed_z;
// 回调函数，用于更新机器人当前位置
void positionCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    if (is_get_start_point)
    {
        return;
    }

    robot_position.x = msg->position.x;
    robot_position.y = msg->position.y;
    robot_position.z = msg->position.z;

    is_get_start_point = true;
}

// 设置起点
void setStartFromRobotPosition(PathPlanner &planner)
{
    planner.setStart(robot_position.x, robot_position.y, robot_position.z);
}

// 设置给定的终点
void setGoal(PathPlanner &planner)
{
    planner.setGoal(goal_x, goal_y, goal_z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_demo");
    ros::NodeHandle nh;
    Visualization visualization(nh, "prm_visual");

    // 从参数服务器获取目标位置
    nh.param("goal_x", goal_x, 12.0);
    nh.param("goal_y", goal_y, 12.0);
    nh.param("goal_z", goal_z, 2.5);

    nh.param("is_2D_planning", is_2D_planning, true);  // 默认 true，表示使用 2D 规划
    nh.param("fixed_z", fixed_z, 1.5);                 // 默认高度 1.5
    
    // 订阅机器人位置
    position_sub = nh.subscribe<mavros_msgs::PositionTarget>("robot_position", 10, positionCallback);

    // 创建路径规划器实例
    PathPlanner planner;

    // 等待接收到位置数据
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();  // 更新回调数据

        if (is_get_start_point)
        {
            // 设置起点和终点
            setStartFromRobotPosition(planner);


            setGoal(planner);

            // 求解路径
            if (planner.solve(2))
            {
                std::cout << "路径规划成功。" << std::endl;

                // 提取最终路径
                auto solution_path = planner.extractSolutionPath();
                visualization.setPathData(solution_path);
            }
            else
            {
                std::cout << "路径规划失败。" << std::endl;
            }

            // 提取图结构中的节点和边
            auto [nodes, edges] = planner.extractGraph();
            visualization.setGraphData(nodes, edges);
            visualization.visualizeStartAndGoal(robot_position.x, robot_position.y, robot_position.z, goal_x, goal_y, goal_z);

            break;  // 退出循环，路径规划完成
        }

        loop_rate.sleep();  // 控制循环频率
    }

    ros::spin();  // 保持节点运行

    return 0;
}
