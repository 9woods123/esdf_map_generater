#include "path_planner/path_planner.h"
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <traj_msgs/Trajectory.h>
#include <traj_msgs/TrajectoryPoint.h>
#include <iostream>
#include <cmath>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
// 全局变量
ros::Subscriber position_sub;
ros::Publisher traj_pub;
geometry_msgs::Point robot_position;  // 存储机器人位置
bool is_get_start_point = true;

// 目标位置
double goal_x, goal_y, goal_z;
bool is_2D_planning = false;
double fixed_z;

// 回调函数：获取机器人当前位置
void positionCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    if (is_get_start_point) return;

    robot_position.x = msg->position.x;
    robot_position.y = msg->position.y;
    robot_position.z = msg->position.z;

    is_get_start_point = true;
}

// 设置起点为当前机器人位置
void setStartFromRobotPosition(PathPlanner &planner)
{
    planner.setStart(-13.5, 10.5, 1.5);
    // planner.setStart(robot_position.x, robot_position.y, robot_position.z);
}

// 设置终点
void setGoal(PathPlanner &planner)
{
    planner.setGoal(goal_x, goal_y, goal_z);
}


std::vector<geometry_msgs::Pose> interpolatePath(
    const std::vector<Node> &path, double step_size)
{
    std::vector<geometry_msgs::Pose> dense_path;

    for (size_t i = 0; i < path.size() - 1; ++i)
    {
        const auto &p1 = path[i];
        const auto &p2 = path[i + 1];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double dz = p2.z - p1.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        int steps = std::ceil(dist / step_size);

        // 计算航向角（朝向角）
        double heading = std::atan2(dy, dx);  // 计算 xy 平面上的朝向角

        for (int s = 0; s < steps; ++s)
        {
            geometry_msgs::Pose pose;
            double t = static_cast<double>(s) / steps;

            // 插值位置
            pose.position.x = p1.x + t * dx;
            pose.position.y = p1.y + t * dy;
            pose.position.z = p1.z + t * dz;

            // 根据航向角计算四元数表示的朝向
            pose.orientation = tf::createQuaternionMsgFromYaw(heading);

            dense_path.push_back(pose);
        }
    }

    // 加入最后一个点的姿态
    geometry_msgs::Pose last_pose;
    last_pose.position.x = path.back().x;
    last_pose.position.y = path.back().y;
    last_pose.position.z = path.back().z;
    last_pose.orientation = tf::createQuaternionMsgFromYaw(
        std::atan2(path.back().y - path[path.size() - 2].y,
                   path.back().x - path[path.size() - 2].x));

    dense_path.push_back(last_pose);

    return dense_path;
}

traj_msgs::Trajectory generateTrajectory(
    const std::vector<Node> &solution_path,
    double step_size,
    double velocity)
{
    traj_msgs::Trajectory traj_msg;
    ros::Time start_time = ros::Time::now();
    traj_msg.header.stamp = start_time;
    traj_msg.header.frame_id = "map";  // 根据实际坐标系修改

    auto dense_points = interpolatePath(solution_path, step_size);
    ros::Duration time_from_start(0.0);

    for (size_t i = 1; i < dense_points.size(); ++i)
    {
        traj_msgs::TrajectoryPoint point;
        point.pose = dense_points[i];

        double dx = dense_points[i].position.x - dense_points[i - 1].position.x;
        double dy = dense_points[i].position.y - dense_points[i - 1].position.y;
        double dz = dense_points[i].position.z - dense_points[i - 1].position.z;
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        double dt = dist / velocity;
        time_from_start += ros::Duration(dt);

        point.header.stamp = start_time + time_from_start;
        point.header.frame_id = traj_msg.header.frame_id;

        traj_msg.points.push_back(point);
    }

    return traj_msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planning_demo");
    ros::NodeHandle nh;
    Visualization visualization(nh, "prm_visual");

    nh.param("goal_x", goal_x, 12.0);
    nh.param("goal_y", goal_y, 12.0);
    nh.param("goal_z", goal_z, 2.5);
    nh.param("is_2D_planning", is_2D_planning, true);
    nh.param("fixed_z", fixed_z, 1.5);

    position_sub = nh.subscribe<mavros_msgs::PositionTarget>("robot_position", 10, positionCallback);
    traj_pub = nh.advertise<traj_msgs::Trajectory>("planned_trajectory", 10);

    PathPlanner planner(is_2D_planning,fixed_z);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        if (is_get_start_point)
        {
            setStartFromRobotPosition(planner);
            setGoal(planner);

            if (planner.solve(2))
            {
                ROS_INFO("===============Successfully================");

                auto solution_path = planner.extractSolutionPath();
                visualization.setPathData(solution_path);

                auto [nodes, edges] = planner.extractGraph();
                visualization.setGraphData(nodes, edges);
                visualization.visualizeStartAndGoal(
                    robot_position.x, robot_position.y, robot_position.z,
                    goal_x, goal_y, goal_z);

                // 生成并发布轨迹
                double step_size = 0.2;
                double velocity = 1.0; // m/s
                auto traj_msg = generateTrajectory(solution_path, step_size, velocity);
                traj_pub.publish(traj_msg);
            }
            else
            {
                ROS_WARN("===================Failed===================");
            }

            break;
        }

        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
