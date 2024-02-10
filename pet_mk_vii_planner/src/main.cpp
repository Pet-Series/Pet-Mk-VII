#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "visualization.hpp"

#include <ugl/lie_group/pose.h>

#include <rclcpp/rclcpp.hpp>

#include <visualization_msgs/msg/marker.hpp>

#include <iostream>
#include <optional>
#include <vector>

namespace pet
{

class RrtSimulation : public rclcpp::Node
{
  public:
    RrtSimulation() : Node("rrt_simulation")
    {
        m_markerPublisher =
            this->create_publisher<visualization_msgs::msg::Marker>("rrt_marker", 10);
        m_markerArrayPublisher =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(
                "rrt_marker_array", 10);
    }

    void runRrt();

  private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_markerPublisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        m_markerArrayPublisher;
};

void RrtSimulation::runRrt()
{
    const rrt::SearchContext searchContext = [] {
        rrt::SearchContext context{};
        context.maxIterations    = 10;
        context.vehicleModel     = rrt::VehicleModel{};
        context.vehicleFootprint = rrt::VehicleFootprint{{-0.02, 0.05}, {0.18, 0.05}};
        context.searchSpace      = rrt::BoundingBox{{-5.0, -5.0}, {5.0, 5.0}};
        context.collisionMap     = rrt::CollisionMap{};
        context.steerFunction    = rrt::steerBezier;
        return context;
    }();

    const rrt::VehicleState startState{ugl::lie::Pose::Identity(), 0.0};
    const ugl::lie::Pose    goalPose{ugl::lie::Rotation::Identity(), {4.0, -1.0, 0.0}};

    rrt::Graph      searchTree{startState};
    const rrt::Goal goal{goalPose};

    std::vector<rrt::Graph>               searchHistory{};
    std::optional<std::vector<rrt::Node>> path{};

    std::cout << "Starting search..." << std::endl;
    for (int i = 0; i < 5000; ++i)
    {
        if (!rclcpp::ok())
        {
            break; // e.g. ctrl-C by user
        }

        path = rrt::search(goal, searchTree, searchContext);

        searchHistory.push_back(searchTree);

        if (path.has_value())
        {
            std::cout << "Goal found!" << std::endl;
            break;
        }
    }

    std::cout << "...search done." << std::endl;

    std::cout << "Starting visualization..." << std::endl;
    resetVisualization(*m_markerPublisher);
    if (path.has_value())
    {
        visualizePath(path.value(), *m_markerPublisher, *m_markerArrayPublisher);
    }
    // visualizeMap(map);
    visualizeSearchTree(searchTree, *m_markerPublisher, *m_markerArrayPublisher);
    std::cout << "...visualization done." << std::endl;
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
