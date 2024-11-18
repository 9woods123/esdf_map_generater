#include "path_planner/path_planner.h"

// 构造函数
PathPlanner::PathPlanner()
{
    // 初始化 SE3 状态空间
    space_ = std::make_shared<ob::SE3StateSpace>();

    // 设置 R^3 部分的边界
    ob::RealVectorBounds bounds(3);
    bounds.setLow(-1); // 下界
    bounds.setHigh(15); // 上界
    space_->setBounds(bounds);

    // 初始化空间信息和问题定义
    si_ = std::make_shared<ob::SpaceInformation>(space_);
    si_->setStateValidityChecker(isStateValid); // 设置状态有效性检查函数
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);

    // 初始化规划器
    planner_ = std::make_shared<og::PRMstar>(si_);
}

// 设置起点
void PathPlanner::setStart(double x, double y, double z)
{
    ob::ScopedState<ob::SE3StateSpace> start(space_);
    start->setXYZ(x, y, z);
    start->rotation().setIdentity(); // 默认旋转为单位四元数
    pdef_->addStartState(start);
}

// 设置终点
void PathPlanner::setGoal(double x, double y, double z)
{
    ob::ScopedState<ob::SE3StateSpace> goal(space_);
    goal->setXYZ(x, y, z);
    goal->rotation().setIdentity(); // 默认旋转为单位四元数
    pdef_->setGoalState(goal);
}

std::pair<std::vector<Node>, std::vector<Edge>> PathPlanner::extractGraph() const
{
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    // 获取 PRMstar 内部的图
    const auto &graph = planner_->getRoadmap();
    std::cout << "Number of vertices: " << boost::num_vertices(graph) << std::endl;
    std::cout << "Number of edges: " << boost::num_edges(graph) << std::endl;

    // 提取节点
    for (auto v : boost::make_iterator_range(boost::vertices(graph)))
    {
        const ob::State *state = boost::get(ompl::geometric::PRM::vertex_state_t(), graph, v);
        if (!state)
        {
            std::cerr << "Failed to get state for vertex " << v << std::endl;
            continue;  // 跳过该节点
        }

        const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
        if (!se3state)
        {
            std::cerr << "Failed to cast state to SE3" << std::endl;
            continue;
        }

        const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
        if (!pos)
        {
            std::cerr << "Failed to extract position from SE3 state" << std::endl;
            continue;
        }

        Node node = {pos->values[0], pos->values[1], pos->values[2]};
        nodes.push_back(node);
    }

    // 提取边
    for (auto e : boost::make_iterator_range(boost::edges(graph)))
    {
        auto from = boost::source(e, graph); 
        auto to = boost::target(e, graph);  

        ob::Cost weight = boost::get(boost::edge_weight_t(), graph, e);
        edges.push_back({from, to});
    }

    return {nodes, edges};
}

// 尝试求解
bool PathPlanner::solve(double time_limit)
{
    planner_->setProblemDefinition(pdef_);
    planner_->setup();

    std::cout << "规划器设置完成，开始求解..." << std::endl;

    // 尝试在指定时间内求解
    ob::PlannerStatus solved = planner_->solve(ob::timedPlannerTerminationCondition(time_limit));

    if (solved)
    {
        ob::PathPtr path = pdef_->getSolutionPath();
        std::cout << "找到解决方案：" << std::endl;
        path->print(std::cout);
        return true;
    }
    else
    {
        std::cout << "未找到解决方案。" << std::endl;
        return false;
    }
}

bool PathPlanner::isStateValid(const ob::State *state)
{
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    std::vector<Obstacle> obstacles = {
        {5.0, 5.0, 0.0, 7},
        {10, -5.0, 0.0, 5},
        {0.0, 10.0, 0.0, 7},
        {0.0, 10.0, 5, 7}
    };

    for (const auto &obstacle : obstacles)
    {
        double dx = pos->values[0] - obstacle.x;
        double dy = pos->values[1] - obstacle.y;
        double dz = pos->values[2] - obstacle.z;

        double distance_sq = dx * dx + dy * dy + dz * dz;
        double radius_sq = obstacle.radius * obstacle.radius;

        if (distance_sq < radius_sq)
        {
            return false;
        }
    }

    return true;
}

// 主函数
int main(int argc, char **argv)
{
    ros::init(argc, argv, "prm_planner");
    ros::NodeHandle nh;

    Visualization visualization(nh, "prm_visual");

    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    // 创建路径规划器实例
    PathPlanner planner;

    // 设置起点和终点
    planner.setStart(0.0, 0.0, 0.0);
    planner.setGoal(15.0, 15.0, 2.0);

    // 求解路径
    if (planner.solve(0.1))
    {
        std::cout << "路径规划成功。" << std::endl;
    }
    else
    {
        std::cout << "路径规划失败。" << std::endl;
    }

    // 提取图结构中的节点和边
    auto [nodes, edges] = planner.extractGraph();
    visualization.setGraphData(nodes, edges);

    ros::spin();

    return 0;
}
