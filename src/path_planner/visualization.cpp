#include "path_planner/visualization.h"

Visualization::Visualization(ros::NodeHandle &nh, const std::string &topic)
    : timer_interval_(0.1) // 默认10Hz
{
    marker_array_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic, 1);
    timer_ = nh.createTimer(ros::Duration(timer_interval_), &Visualization::timerCallback, this);
}

void Visualization::setGraphData(const std::vector<Node> &nodes, const std::vector<Edge> &edges)
{
    nodes_ = nodes; // 存储节点数据
    edges_ = edges; // 存储边数据
}

void Visualization::timerCallback(const ros::TimerEvent &)
{
    // 如果没有图数据，则跳过发布
    if (nodes_.empty() && edges_.empty())
    {
        return;
    }

    visualization_msgs::MarkerArray marker_array;

    // 节点可视化
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

    // 边可视化
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

    // 发布 MarkerArray
    marker_array_pub_.publish(marker_array);
}
