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

void ESDFMapGenerator::getMapBounds(double &min_x, double &min_y, double &min_z, 
    double &max_x, double &max_y, double &max_z)
{
// 假设 ESDF 生成器已经有 tree_，从 tree_ 里获取边界
tree_.getMetricMin(min_x, min_y, min_z);
tree_.getMetricMax(max_x, max_y, max_z);

}


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
    //大概率是这里少了点，由于free space的合并？

    ROS_INFO("Extracted %zu occupied points, %zu free points", octo_cloud_->size(), freespace_cloud_->size());

    // 生成 ESDF
    generateESDF();
    convertOctomapToRosMsg(); 
    convertEsdfToPointCloudMsg();
}


// void ESDFMapGenerator::generateESDF() {

//     pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution_);
//     octree.setInputCloud(octo_cloud_);
//     octree.addPointsFromInputCloud();

//     // for (const auto& free_point : freespace_cloud_->points) {
//     //     std::vector<int> point_idx;
//     //     std::vector<float> point_dist_sq;


//     //     if (octree.nearestKSearch(free_point, 1, point_idx, point_dist_sq) > 0) {
//     //         float distance = std::sqrt(point_dist_sq[0]);
//     //         VoxelID voxel_id{
//     //             static_cast<int>(std::floor(free_point.x / resolution_)),
//     //             static_cast<int>(std::floor(free_point.y / resolution_)),
//     //             static_cast<int>(std::floor(free_point.z / resolution_))};

//     //         esdf_map_[voxel_id] = distance;
//     //         // if ( 0<voxel_id.x && voxel_id.x<50 && 0<voxel_id.y && voxel_id.y<50 )
//     //         // {
//     //         //     std::cout<<"voxel_id: "<<voxel_id.x<<", "<<voxel_id.y<<", "<<voxel_id.z<<std::endl;
//     //         // }

//     //     }

//     // }

//     // 获取边界范围
//     double min_x, min_y, min_z, max_x, max_y, max_z;
//     tree_.getMetricMin(min_x, min_y, min_z);
//     tree_.getMetricMax(max_x, max_y, max_z);

//     int min_x_idx = static_cast<int>(std::floor(min_x / resolution_));
//     int max_x_idx = static_cast<int>(std::floor(max_x / resolution_));
//     int min_y_idx = static_cast<int>(std::floor(min_y / resolution_));
//     int max_y_idx = static_cast<int>(std::floor(max_y / resolution_));
//     int min_z_idx = static_cast<int>(std::floor(min_z / resolution_));
//     int max_z_idx = static_cast<int>(std::floor(max_z / resolution_));

//     std::cout << "Voxel Index Range:" << std::endl;
//     std::cout << "  X: [" << min_x_idx << ", " << max_x_idx << "]" << std::endl;
//     std::cout << "  Y: [" << min_y_idx << ", " << max_y_idx << "]" << std::endl;
//     std::cout << "  Z: [" << min_z_idx << ", " << max_z_idx << "]" << std::endl;
    
//     std::cout << "Real-world Boundaries:" << std::endl;
//     std::cout << "  X: [" << min_x << ", " << max_x << "]" << std::endl;
//     std::cout << "  Y: [" << min_y << ", " << max_y << "]" << std::endl;
//     std::cout << "  Z: [" << min_z << ", " << max_z << "]" << std::endl;
    
//     for (int x = min_x_idx; x <= max_x_idx; ++x) {
//         for (int y = min_y_idx; y <= max_y_idx; ++y) {
//             for (int z = min_z_idx; z <= max_z_idx; ++z) {
//                 pcl::PointXYZ query_point(
//                     (x + 0.5f) * resolution_,
//                     (y + 0.5f) * resolution_,
//                     (z + 0.5f) * resolution_
//                 );

//                 std::vector<int> point_idx;
//                 std::vector<float> point_dist_sq;

//                 if (octree.nearestKSearch(query_point, 1, point_idx, point_dist_sq) > 0) {
//                     float distance = std::sqrt(point_dist_sq[0]);
//                     VoxelID voxel_id{x, y, z};
//                     esdf_map_[voxel_id] = distance;
//                 }
//             }
//         }
//     }


//     ROS_INFO("Generated ESDF with %zu entries", esdf_map_.size());
// }

void ESDFMapGenerator::generateESDF() {
    using namespace std;

    esdf_map_.clear();

    // 初始化 Octree
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution_);
    octree.setInputCloud(octo_cloud_);
    octree.addPointsFromInputCloud();

    // 用于 BFS 的队列和方向
    queue<VoxelID> bfs_queue;
    std::vector<VoxelID> directions = {
        {1, 0, 0}, {-1, 0, 0},
        {0, 1, 0}, {0, -1, 0},
        {0, 0, 1}, {0, 0, -1}
    };

    // Step 1: 初始化起点（障碍物体素）
    for (const auto& point : octo_cloud_->points) {
        VoxelID voxel_id{
            static_cast<int>(std::floor(point.x / resolution_)),
            static_cast<int>(std::floor(point.y / resolution_)),
            static_cast<int>(std::floor(point.z / resolution_))
        };
        esdf_map_[voxel_id] = 0.0f; // 距离为 0
        bfs_queue.push(voxel_id);
    }

    // Step 2: 广度优先搜索扩展
    while (!bfs_queue.empty()) {
        VoxelID current = bfs_queue.front();
        bfs_queue.pop();
        float current_dist = esdf_map_[current];

        for (const auto& dir : directions) {
            VoxelID neighbor{
                current.x + dir.x,
                current.y + dir.y,
                current.z + dir.z
            };

            // 如果该 voxel 已计算过距离，跳过
            if (esdf_map_.find(neighbor) != esdf_map_.end())
                continue;

            // 获取邻居 voxel 的中心坐标（真实世界坐标）
            pcl::PointXYZ neighbor_point{
                (neighbor.x + 0.5f) * resolution_,
                (neighbor.y + 0.5f) * resolution_,
                (neighbor.z + 0.5f) * resolution_
            };

            // 最近障碍点搜索（使用 Octree）
            std::vector<int> point_idx;
            std::vector<float> point_dist_sq;
            if (octree.nearestKSearch(neighbor_point, 1, point_idx, point_dist_sq) > 0) {
                float distance = std::sqrt(point_dist_sq[0]);
                esdf_map_[neighbor] = distance;

                // 可选：只在一定范围内传播，提升效率
                if (distance < 0.5f) // 0.5 米内传播
                    bfs_queue.push(neighbor);
            }
        }
    }

    ROS_INFO("Generated ESDF with %zu entries using BFS", esdf_map_.size());
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

        // // 筛选出大于 maximum_distance 的点
        // if (distance > maximum_distance) {
        //     continue;  // 如果距离大于最大距离，跳过该点
        // }

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
        ROS_ERROR("Point (%f, %f, %f) not found in ESDF map. with ID %i,%i,%i", x, y, z,
        voxel_id.x,voxel_id.y,voxel_id.z);
        min_distance=999;
        gradient.setZero();
        return true;  // 如果该点没有在 ESDF 中，返回 true
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

bool ESDFMapGenerator::isPointOccupied(double x, double y, double z) {
    // 获取目标点的 3D 坐标
    octomap::point3d query_point(x, y, z);

    // 查找目标点所在的节点
    octomap::OcTreeNode* node = tree_.search(query_point);
    if (node != nullptr && tree_.isNodeOccupied(node)) {
        return true;  // 如果目标点已经占用，直接返回碰撞
    }

    // 使用 26 邻域来查找周围的节点
    // 获取一个小范围内的所有节点
    std::vector<octomap::OcTreeNode*> neighbors;
    float radius = 3;  // 使用碰撞阈值作为搜索半径
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dz = -1; dz <= 1; ++dz) {
                // 排除中心点自己
                if (dx == 0 && dy == 0 && dz == 0) continue;

                // 计算邻域点坐标
                octomap::point3d neighbor_point(x + dx * tree_.getResolution(),
                                                y + dy * tree_.getResolution(),
                                                z + dz * tree_.getResolution());
                
                // 查找邻域节点
                octomap::OcTreeNode* neighbor_node = tree_.search(neighbor_point);
                if (neighbor_node != nullptr && tree_.isNodeOccupied(neighbor_node)) {
                        return true;  // 如果有邻域节点的距离小于阈值，视为碰撞
                    }
                }
            }
        }

    // // 如果没有发现碰撞，返回 false
    return false;
}

bool ESDFMapGenerator::isPointOccupiedWithVolume(double x, double y, double z, double radius) {
    // 获取目标点的 3D 坐标

    // 遍历目标点附近的节点
    for (float dx = -radius; dx <= radius; dx += tree_.getResolution()) {
        for (float dy = -radius; dy <= radius; dy += tree_.getResolution()) {
            for (float dz = -radius; dz <= radius; dz += tree_.getResolution()) {
                // 计算检测点的坐标
                octomap::point3d check_point(x + dx, y + dy, z + dz);

                // 查找检测点的节点
                octomap::OcTreeNode* node = tree_.search(check_point);
                if (node != nullptr && tree_.isNodeOccupied(node)) {
                    // 如果检测点被占用，视为碰撞
                    return true;
                }
            }
        }
    }

    // 如果所有检测点都未被占用，返回未碰撞
    return false;
}


void ESDFMapGenerator::publishCallback(const ros::TimerEvent&) {
    ros::Time now = ros::Time::now();
    octomap_msg_.header.stamp = now;
    esdf_msg_.header.stamp = now;

    octomap_pub_.publish(octomap_msg_);
    esdf_pub_.publish(esdf_msg_);

    ROS_INFO("Published OctoMap and ESDF.");

}
}