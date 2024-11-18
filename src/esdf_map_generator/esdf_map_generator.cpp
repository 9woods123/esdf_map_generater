#include "esdf_map_generator/esdf_map_generator.h"
namespace EsdfMap{

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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr 
    esdf_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    
    // 设置最大距离值，超过此距离的点不添加
    float maximum_distance = 2.0f;  // 自定义的最大距离
    // TODO maximum_distance is a param
    float max_distance = 0.0f;
    
    // 遍历 ESDF 地图，筛选距离并应用渐变颜色
    for (const auto& kv : esdf_map_) {
        const VoxelID& voxel = kv.first;
        float distance = kv.second;

        // 筛选出大于 maximum_distance 的点
        if (distance > maximum_distance) {
            continue;  // 如果距离大于最大距离，跳过该点
        }
        if (voxel.z * resolution_ < 2.5f || voxel.z * resolution_ > 3.0f) {
            continue;  // 如果 z 值不在 2.5 到 3 之间，跳过
        }

        pcl::PointXYZRGB point;
        point.x = voxel.x * resolution_;
        point.y = voxel.y * resolution_;
        point.z = voxel.z * resolution_;

        // 归一化距离
        float normalized_distance = distance / maximum_distance;  // 归一化距离

        // 计算颜色（距离越小越红，距离越大越蓝，采用红橙黄绿青蓝渐变）
        int r = 0, g = 0, b = 0;
        
        // 红色 -> 蓝色渐变（通过 HSV 色彩空间实现）
        if (normalized_distance <= 0.5f) {
            r = static_cast<int>((1.0f - 2.0f * normalized_distance) * 255);   // 红色
            g = static_cast<int>((2.0f * normalized_distance) * 255);           // 黄色到绿色
            b = 0;                                                              // 蓝色部分
        } else {
            r = 0;                                                               // 红色部分
            g = static_cast<int>((2.0f * (1.0f - normalized_distance)) * 255);   // 绿色到青色
            b = static_cast<int>((2.0f * (normalized_distance - 0.5f)) * 255);   // 蓝色
        }

        // 设置点的颜色
        point.r = r;
        point.g = g;
        point.b = b;

        // 添加点到点云
        esdf_cloud->push_back(point);
    }

    // 将点云转换为 ROS 消息
    pcl::toROSMsg(*esdf_cloud, esdf_msg_);
    esdf_msg_.header.frame_id = "map";
}

bool ESDFMapGenerator::getMinCollisionDistanceAndGradient(float x, float y, float z, float& min_distance, Eigen::Vector3f& gradient) {
    // 查找 (x, y, z) 所在位置的最小碰撞距离和梯度
    VoxelID voxel_id{
        static_cast<int>(std::floor(x / resolution_)),
        static_cast<int>(std::floor(y / resolution_)),
        static_cast<int>(std::floor(z / resolution_))
    };

    // 检查该点是否在 ESDF 中
    auto it = esdf_map_.find(voxel_id);
    if (it == esdf_map_.end()) {
        ROS_ERROR("Point (%f, %f, %f) not found in ESDF map.", x, y, z);
        return false;  // 如果该点没有在 ESDF 中，返回 false
    }

    // 获取该点的最小碰撞距离
    min_distance = it->second;

    // 计算梯度（距离场的梯度近似）
    gradient.setZero();

    // 设置一个小的偏移量，查找相邻点
    float offset = resolution_;  // 偏移量等于分辨率
    
    // 26邻域
    int directions[26][3] = {
        {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
        {1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0}, {1, 0, 1}, {-1, 0, 1},
        {1, 0, -1}, {-1, 0, -1}, {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1},
        {1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1}, {-1, 1, -1},
        {1, -1, -1}, {-1, -1, -1}
    };

    // 使用 3x3x3 邻域（26 个邻居）计算梯度
    for (int i = 0; i < 26; ++i) {
        // 对每个邻居点应用偏移量
        VoxelID neighbor_id = voxel_id;
        neighbor_id.x += directions[i][0];
        neighbor_id.y += directions[i][1];
        neighbor_id.z += directions[i][2];

        // 查找邻居点的距离
        auto neighbor_it = esdf_map_.find(neighbor_id);
        if (neighbor_it != esdf_map_.end()) {
            float neighbor_distance = neighbor_it->second;

            // 根据邻居与当前点的距离差计算梯度
            float distance_diff = neighbor_distance - min_distance;
            if (directions[i][0] != 0) gradient.x() += distance_diff;
            if (directions[i][1] != 0) gradient.y() += distance_diff;
            if (directions[i][2] != 0) gradient.z() += distance_diff;
        }
    }

    // 梯度归一化
    gradient.x() /= offset;
    gradient.y() /= offset;
    gradient.z() /= offset;

    return true;
}

void ESDFMapGenerator::publishCallback(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    octomap_msg_.header.stamp = now;
    esdf_msg_.header.stamp = now;

    octomap_pub_.publish(octomap_msg_);
    esdf_pub_.publish(esdf_msg_);

    ROS_INFO("Published OctoMap and ESDF.");


    // float x = 0.0f, y = 0.0f, z = 2.0f;
    // float min_distance;
    // Eigen::Vector3f gradient;

    // // 使用 std::chrono 计时
    // auto start_time = std::chrono::high_resolution_clock::now();

    // if (getMinCollisionDistanceAndGradient(x, y, z, min_distance, gradient)) {
    //     // 计算耗时
    //     auto end_time = std::chrono::high_resolution_clock::now();
    //     std::chrono::duration<float> duration = end_time - start_time;
    //     ROS_INFO("Min Collision Distance: %f", min_distance);
    //     ROS_INFO("Gradient: [%f, %f, %f]", gradient.x(), gradient.y(), gradient.z());
    //     ROS_INFO("Time taken: %f seconds", duration.count()); // 打印执行时间
    // } else {
    //     ROS_ERROR("Failed to get distance and gradient for point (%f, %f, %f)", x, y, z);
    // }
    // 0.04 ms per require

}
}