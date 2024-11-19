#include "path_planner/path_planner.h"

// 构造函数
PathPlanner::PathPlanner()
{



    // 初始化 SE3 状态空间
    space_ = std::make_shared<ob::SE3StateSpace>();

    // 设置 R^3 部分的长方形边界
    ob::RealVectorBounds bounds(3);

    // 设置每个维度的边界
    bounds.setLow(0, -15); // x 维度下界
    bounds.setHigh(0, 15); // x 维度上界

    bounds.setLow(1, -15); // y 维度下界
    bounds.setHigh(1, 15); // y 维度上界

    bounds.setLow(2, -2);   // z 维度下界
    bounds.setHigh(2, 3);  // z 维度上界

    space_->setBounds(bounds);


    // 初始化空间信息和问题定义
    si_ = std::make_shared<ob::SpaceInformation>(space_);
    si_->setStateValidityChecker(isStateValid); // 设置状态有效性检查函数
    pdef_ = std::make_shared<ob::ProblemDefinition>(si_);

    // 初始化规划器
    planner_ = std::make_shared<og::PRMstar>(si_);

    esdf_map_generator_=std::make_shared<EsdfMap::ESDFMapGenerator>();

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

std::vector<Node> PathPlanner::extractSolutionPath() const
{
    std::vector<Node> path_points;

    // 检查是否有解决方案
    if (pdef_->hasSolution())
    {
        // 获取路径
        auto path = pdef_->getSolutionPath();
        auto geometric_path = path->as<og::PathGeometric>();
        if (!geometric_path)
        {
            std::cerr << "Failed to cast solution path to PathGeometric" << std::endl;
            return {};
        }

        // 提取每个状态的坐标
        for (size_t i = 0; i < geometric_path->getStateCount(); ++i)
        {
            const ob::State *state = geometric_path->getState(i);
            const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
            const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

            Node point = {pos->values[0], pos->values[1], pos->values[2]};
            path_points.push_back(point);
        }
    }
    else
    {
        std::cerr << "No solution path available" << std::endl;
    }

    return path_points;
}

bool PathPlanner::isStateValid(const ob::State *state)
{
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    float current_state_x = pos->values[0];
    float current_state_y = pos->values[1];
    float current_state_z = pos->values[2];

    float distance;
    Eigen::Vector3f gradient;
    float safe_dist=0.5;
    if(esdf_map_generator_->getMinCollisionDistanceAndGradient(
        current_state_x,current_state_y,current_state_z,distance,gradient))
    {
        if(distance>safe_dist)
        {
            return true;
        }

    }
    return false;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prm_planner");
    ros::NodeHandle nh;
    Visualization visualization(nh, "prm_visual");

    // 从 ROS 参数服务器获取 start 和 goal 的位置
    double start_x, start_y, start_z;
    double goal_x, goal_y, goal_z;
    // 获取参数，如果没有则使用默认值
    
    nh.param("start_x", start_x, -10.0);
    nh.param("start_y", start_y, -6.0);
    nh.param("start_z", start_z, 1.5);

    nh.param("goal_x", goal_x, 12.0);
    nh.param("goal_y", goal_y, 12.0);
    nh.param("goal_z", goal_z, 2.5);

    // 创建路径规划器实例
    PathPlanner planner;

    // 设置起点和终点
    planner.setStart(start_x, start_y, start_z);
    planner.setGoal(goal_x, goal_y, goal_z);



    if (planner.solve(20.0))
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
    visualization.visualizeStartAndGoal(start_x,start_y,start_z,goal_x,goal_y,goal_z);

    ros::spin();

    return 0;
}
