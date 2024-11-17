#include "esdf_map_generator/esdf_map_generator.h"

ESDFMapGenerator::ESDFMapGenerator()
    : tree_(0.1),nh_private_("~") {


    nh_private_.param("octomap_file", octomap_file_, std::string("/path/to/default/octomap.bt"));

    octo_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    freespace_cloud_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr esdf_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 1, true);
    pointcloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("octomap_pointcloud", 1, true);
    esdf_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("esdf_pointcloud", 1, true);
    timer_ = nh_.createTimer(ros::Duration(1.0), &ESDFMapGenerator::publishCallback, this);
    
    loadOctomap();
}

ESDFMapGenerator::~ESDFMapGenerator() {}

void ESDFMapGenerator::loadOctomap() {

    if (!tree_.readBinary(octomap_file_)) {
        ROS_ERROR("Failed to load OctoMap from file: %s", octomap_file_.c_str());
        return;
    }

    // 从 OctoMap 中获取分辨率
    resolution_ = tree_.getResolution();
    ROS_INFO("Loaded OctoMap with %zu nodes at resolution %f", tree_.size(), resolution_);

    // 遍历 OctoMap 的节点
    for (octomap::OcTree::iterator it = tree_.begin(); it != tree_.end(); ++it) {
        pcl::PointXYZ point(it.getX(), it.getY(), it.getZ());
        if (tree_.isNodeOccupied(*it)) {
            octo_cloud_->push_back(point); // 占用空间
        } else {
            freespace_cloud_->push_back(point); // 自由空间
        }
    }
    ROS_INFO("Extracted %zu occupied points, %zu free points", octo_cloud_->size(), freespace_cloud_->size());

    // 生成 ESDF
    generateESDF();
    convertOctomapToRosMsg(); 
    convertEsdfToPointCloudMsg();
}



void ESDFMapGenerator::generateESDF() {
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution_);
    octree.setInputCloud(octo_cloud_);
    octree.addPointsFromInputCloud();

    for (const auto& free_point : freespace_cloud_->points) {
        std::vector<int> point_idx;
        std::vector<float> point_dist_sq;
        if (octree.nearestKSearch(free_point, 1, point_idx, point_dist_sq) > 0) {
            float distance = std::sqrt(point_dist_sq[0]);
            VoxelID voxel_id{
                static_cast<int>(std::floor(free_point.x / resolution_)),
                static_cast<int>(std::floor(free_point.y / resolution_)),
                static_cast<int>(std::floor(free_point.z / resolution_))};
            esdf_map_[voxel_id] = distance;
        }
    }
    ROS_INFO("Generated ESDF with %zu entries", esdf_map_.size());
}


// 将 OctoMap 转换为 ROS 消息
void ESDFMapGenerator::convertOctomapToRosMsg() {
    if (!octomap_msgs::fullMapToMsg(tree_, octomap_msg_)) {
        ROS_ERROR("Failed to convert OctoMap to ROS message!");
        return;
    }
    octomap_msg_.header.frame_id = "map";
}


// 将 ESDF 转换为带颜色的 ROS 点云消息
void ESDFMapGenerator::convertEsdfToPointCloudMsg() {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr esdf_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    // 设置最大距离值以做颜色归一化
    float max_distance = 0.0f;
    for (const auto& kv : esdf_map_) {
        max_distance = std::max(max_distance, kv.second);
    }

    for (const auto& kv : esdf_map_) {
        const VoxelID& voxel = kv.first;
        float distance = kv.second;

        pcl::PointXYZRGB point;
        point.x = voxel.x * resolution_;
        point.y = voxel.y * resolution_;
        point.z = voxel.z * resolution_;

        // 计算颜色（distance 越小，越红色）
        float normalized_distance = distance / max_distance; // 归一化距离
        int red = static_cast<int>((1.0f - normalized_distance) * 255);  // 距离越小越红
        int green = 0;  // 保持绿色为 0
        int blue = static_cast<int>(normalized_distance * 255);  // 距离越大越蓝

        point.r = red;
        point.g = green;
        point.b = blue;

        esdf_cloud->push_back(point);
    }

    // 将点云转换为 ROS 消息
    pcl::toROSMsg(*esdf_cloud, esdf_msg_);
    esdf_msg_.header.frame_id = "map";
}


void ESDFMapGenerator::publishCallback(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    octomap_msg_.header.stamp = now;
    esdf_msg_.header.stamp = now;

    octomap_pub_.publish(octomap_msg_);
    esdf_pub_.publish(esdf_msg_);

    ROS_INFO("Published OctoMap and ESDF.");
}