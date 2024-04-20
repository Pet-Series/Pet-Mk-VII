#include "pathVerification.hpp"
#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "pet_mk_vii_planner/steerBezier.hpp"
#include "pet_mk_vii_planner/steerCtrv.hpp"
#include "rviz_visualizer.hpp"
#include "utility/tiktok.hpp"

#include <ugl/lie_group/pose.h>

#include <rclcpp/node.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <optional>
#include <vector>

namespace pet
{

void printSearchInfo(const rrt::SearchDiagnostics &diag)
{
    std::cout << std::endl;
    std::cout << "Search diagnostics:" << std::endl;
    std::cout << "  Connections: " << diag.totalConnections << "/" << diag.totalIterations
              << std::endl;
}

class RrtSimulation : public rclcpp::Node
{
  public:
    RrtSimulation();

    void runRrt();

    rrt::Goal         loadGoalPose() const;
    rrt::VehicleModel loadVehicleModel() const;

  private:
    RvizVisualizer m_visualizer;
};

RrtSimulation::RrtSimulation() : Node("rrt_simulation"), m_visualizer(*this)
{
    declare_parameter("goal.x", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("goal.y", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("goal.yaw", rclcpp::PARAMETER_DOUBLE);

    declare_parameter("max_speed", rclcpp::PARAMETER_DOUBLE);
    declare_parameter("max_curvature", rclcpp::PARAMETER_DOUBLE);
}

void RrtSimulation::runRrt()
{
    const rrt::SearchContext searchContext = [this] {
        rrt::SearchContext context{};
        context.maxIterations    = 100;
        context.vehicleModel     = loadVehicleModel();
        context.vehicleFootprint = rrt::VehicleFootprint{{-0.02, 0.05}, {0.18, 0.05}};
        context.searchSpace      = rrt::BoundingBox{{-5.0, -5.0}, {5.0, 5.0}};
        context.collisionMap     = rrt::CollisionMap{};
        context.steerFunction    = rrt::steerBezierPath;
        return context;
    }();

    const rrt::VehicleState startState{ugl::lie::Pose::Identity(), 0.0};
    const rrt::Goal         goal = loadGoalPose();

    rrt::Graph searchTree{startState, searchContext.searchSpace};

    std::vector<rrt::Graph>               searchHistory{};
    std::optional<std::vector<rrt::Node>> path{};

    rrt::SearchDiagnostics aggregatedDiag{};

    std::cout << "Starting search..." << std::endl;
    for (int i = 0; i < 500; ++i)
    {
        if (!rclcpp::ok())
        {
            break; // e.g. ctrl-C by user
        }

        const auto result = rrt::search(goal, searchTree, searchContext);
        aggregatedDiag += result.diag;

        searchHistory.push_back(searchTree);

        if (result.path.has_value())
        {
            path = result.path;
            std::cout << "Goal found!" << std::endl;
            break;
        }
    }
    std::cout << "...search done." << std::endl;

    std::cout << "Starting visualization..." << std::endl;
    m_visualizer.resetVisualization();
    // visualizeMap(map);
    m_visualizer.visualizeSearchTree(searchTree);
    if (path.has_value())
    {
        m_visualizer.visualizePath(path.value());
    }
    std::cout << "...visualization done." << std::endl;

    std::cout << "Starting path verification..." << std::endl;
    // verifyVelocityContinuity(searchTree);
    // verifyHeadingContinuity(searchTree);
    std::cout << "...path verification done." << std::endl;

    printSearchInfo(aggregatedDiag);

    util::TikTok::printData();
}

rrt::Goal RrtSimulation::loadGoalPose() const
{
    const double x   = get_parameter("goal.x").as_double();
    const double y   = get_parameter("goal.y").as_double();
    const double yaw = get_parameter("goal.yaw").as_double();

    const ugl::Vector3        goalPosition{x, y, 0.0};
    const ugl::UnitQuaternion goalOrientation{
        Eigen::AngleAxisd{yaw, ugl::Vector3::UnitZ()}};

    const ugl::lie::Pose goalPose{goalOrientation, goalPosition};

    return rrt::Goal{goalPose};
}

rrt::VehicleModel RrtSimulation::loadVehicleModel() const
{
    const double maxSpeed     = get_parameter("max_speed").as_double();
    const double maxCurvature = get_parameter("max_curvature").as_double();
    return rrt::VehicleModel{maxSpeed, maxCurvature};
}

} // namespace pet

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const auto rosNode = std::make_shared<pet::RrtSimulation>();
    rosNode->runRrt();

    rclcpp::shutdown();
    return 0;
}
