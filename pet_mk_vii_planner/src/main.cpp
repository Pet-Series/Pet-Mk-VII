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
    RrtSimulation() : Node("rrt_simulation"), m_visualizer(*this) {}

    void runRrt();

    rrt::Goal loadGoalPose() const;

  private:
    RvizVisualizer m_visualizer;
};

void RrtSimulation::runRrt()
{
    const rrt::SearchContext searchContext = [] {
        rrt::SearchContext context{};
        context.maxIterations    = 100;
        context.vehicleModel     = rrt::VehicleModel{};
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
    // const ugl::Vector3 goalPosition{4.0, -1.0, 0.0};
    // const ugl::Vector3 goalPosition{4.0, 1.0, 0.0};
    // const ugl::Vector3 goalPosition{-4.0, 1.0, 0.0};
    // const ugl::Vector3 goalPosition{0.0, 4.0, 0.0};
    const ugl::Vector3 goalPosition{2.0, 4.0, 0.0};

    // const double goalHeading = 0.0;
    // const double goalHeading = M_PI;
    const double goalHeading = -M_PI / 2;

    const ugl::UnitQuaternion goalOrientation{
        Eigen::AngleAxisd{goalHeading, ugl::Vector3::UnitZ()}};
    const ugl::lie::Pose goalPose{goalOrientation, goalPosition};

    return rrt::Goal{goalPose};
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
