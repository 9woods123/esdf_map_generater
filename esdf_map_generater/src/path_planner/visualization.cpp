#include "path_planner/visualization.h"

Visualization::Visualization(ros::NodeHandle &nh, const std::string &topic)
    : timer_interval_(0.1) // 默认10Hz
{
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
    timer_ = nh.createTimer(ros::Duration(timer_interval_), &Visualization::timerCallback, this);
    pub_start_goal = nh.advertise<visualization_msgs::Marker>("visualization_start_goal", 1); // 初始化 pub_start_goal

}

void Visualization::setGraphData(const std::vector<Node> &nodes, const std::vector<Edge> &edges)
{
    nodes_ = nodes; // 存储节点数据
    edges_ = edges; // 存储边数据
}

void Visualization::setPathData(const std::vector<Node> &path_points)
{
    path_ = path_points; // 存储路径点数据
}

void Visualization::timerCallback(const ros::TimerEvent &)
{

    visualization_msgs::MarkerArray marker_array;

    // 节点可视化
    if (!nodes_.empty())
    {
        visualization_msgs::Marker node_marker;
        node_marker.header.frame_id = "map";
        node_marker.header.stamp = ros::Time::now();
        node_marker.ns = "prm_graph";
        node_marker.id = 0;
        node_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        node_marker.action = visualization_msgs::Marker::ADD;
        node_marker.scale.x = 0.2;
        node_marker.scale.y = 0.2;
        node_marker.scale.z = 0.2;
        node_marker.color.r = 0.0;
        node_marker.color.g = 1.0;
        node_marker.color.b = 0.0;
        node_marker.color.a = 1.0;

        for (const auto &node : nodes_)
        {
            geometry_msgs::Point p;
            p.x = node.x;
            p.y = node.y;
            p.z = node.z;
            node_marker.points.push_back(p);
        }
        marker_array.markers.push_back(node_marker);
    }

    // 边可视化
    if (!edges_.empty())
    {
        visualization_msgs::Marker edge_marker;
        edge_marker.header.frame_id = "map";
        edge_marker.header.stamp = ros::Time::now();
        edge_marker.ns = "prm_graph";
        edge_marker.id = 1;
        edge_marker.type = visualization_msgs::Marker::LINE_LIST;
        edge_marker.action = visualization_msgs::Marker::ADD;
        edge_marker.scale.x = 0.05;
        edge_marker.color.r = 1.0;
        edge_marker.color.g = 0.0;
        edge_marker.color.b = 0.0;
        edge_marker.color.a = 1.0;

        for (const auto &edge : edges_)
        {
            geometry_msgs::Point p1, p2;
            p1.x = nodes_[edge.from].x;
            p1.y = nodes_[edge.from].y;
            p1.z = nodes_[edge.from].z;

            p2.x = nodes_[edge.to].x;
            p2.y = nodes_[edge.to].y;
            p2.z = nodes_[edge.to].z;

            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);
        }
        marker_array.markers.push_back(edge_marker);
    }

    // 路径可视化
    if (!path_.empty())
    {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "map";
        path_marker.header.stamp = ros::Time::now();
        path_marker.ns = "path";
        path_marker.id = 2;
        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;
        path_marker.scale.x = 0.5; // 线宽
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 0.0;
        path_marker.color.a = 1.0;

        for (const auto &node : path_)
        {
            geometry_msgs::Point p;
            p.x = node.x;
            p.y = node.y;
            p.z = node.z;
            path_marker.points.push_back(p);

        }
        marker_array.markers.push_back(path_marker);
    }

    // 发布所有 MarkerArray
    marker_array_pub_.publish(marker_array);


}


void Visualization::visualizeStartAndGoal(double start_x, double start_y, double start_z,
                                           double goal_x, double goal_y, double goal_z)
{
    // 可视化起点
    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "map";  // 或者其他框架
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "start_goal";
    start_marker.id = 0;  // 起点的 ID 为 0
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.position.x = start_x;
    start_marker.pose.position.y = start_y;
    start_marker.pose.position.z = start_z;
    start_marker.scale.x = 1.0;
    start_marker.scale.y = 1.0;
    start_marker.scale.z = 1.0;
    start_marker.color.a = 1.0;
    start_marker.color.r = 1.0;  // 红色表示起点

    pub_start_goal.publish(start_marker);

    // 可视化终点
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map";  // 或者其他框架
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "start_goal";
    goal_marker.id = 1;  // 终点的 ID 为 1
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = goal_x;
    goal_marker.pose.position.y = goal_y;
    goal_marker.pose.position.z = goal_z;
    goal_marker.scale.x = 1.0;
    goal_marker.scale.y = 1.0;
    goal_marker.scale.z = 1.0;
    goal_marker.color.a = 1.0;
    goal_marker.color.g = 1.0;  // 绿色表示终点

    pub_start_goal.publish(goal_marker);
}
