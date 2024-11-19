#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/config.h>
#include <iostream>
#include "visualization.h"
#include "ompl/geometric/planners/prm/PRM.h"

#include "esdf_map_generator/esdf_map_generator.h"

namespace ob = ompl::base;
namespace og = ompl::geometric;

using namespace ompl::geometric;

struct Obstacle
{
    double x, y, z;  // 障碍物的中心坐标
    double radius;    // 障碍物的半径
};

class PathPlanner
{
public:
    PathPlanner();

    /**
     * @brief 设置规划起点
     * @param x 起点的 X 坐标
     * @param y 起点的 Y 坐标
     * @param z 起点的 Z 坐标
     */
    void setStart(double x, double y, double z);

    /**
     * @brief 设置规划终点
     * @param x 终点的 X 坐标
     * @param y 终点的 Y 坐标
     * @param z 终点的 Z 坐标
     */
    void setGoal(double x, double y, double z);

    /**
     * @brief 尝试在指定时间内求解路径
     * @param time_limit 求解时间限制（秒）
     * @return 如果找到路径返回 true，否则返回 false
     */
    bool solve(double time_limit);
    
    std::pair<std::vector<Node>, std::vector<Edge>> extractGraph() const;
    std::vector<Node> extractSolutionPath() const;

private:
    /**
     * @brief 状态有效性检查函数
     * @param state 要检查的状态
     * @return 如果状态有效返回 true，否则返回 false
     */
    static bool isStateValid(const ob::State *state);


    std::shared_ptr<ob::SE3StateSpace> space_;       ///< 状态空间
    std::shared_ptr<ob::SpaceInformation> si_;       ///< 空间信息
    std::shared_ptr<ob::ProblemDefinition> pdef_;    ///< 问题定义
    std::shared_ptr<og::PRMstar> planner_;


    static inline std::shared_ptr<EsdfMap::ESDFMapGenerator> esdf_map_generator_;

    // static inline EsdfMap::ESDFMapGenerator esdf_map_generator_; ///< ESDF 地图生成器实例
    static constexpr double collision_threshold = 0.1;           ///< 碰撞阈值


};


#endif // PATH_PLANNER_H
