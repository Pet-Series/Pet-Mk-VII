#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"
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
            this->create_publisher<visualization_msgs::msg::Marker>("rrt_found_path", 10);
    }

    void runRrt();

  private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_markerPublisher;
};

void RrtSimulation::runRrt()
{
    const rrt::VehicleFootprint footprint{{-0.02, 0.05}, {0.18, 0.05}};
    const rrt::BoundingBox      searchSpace{{-5.0, -5.0}, {5.0, 5.0}};
    const rrt::CollisionMap     map{};
    const rrt::SearchContext    context{10, footprint, searchSpace, map};

    const ugl::lie::Pose startPose = ugl::lie::Pose::Identity();
    const ugl::lie::Pose goalPose{ugl::lie::Rotation::Identity(), {4.0, -1.0, 0.0}};

    rrt::Graph      searchTree{startPose};
    const rrt::Goal goal{goalPose};

    std::vector<rrt::Graph> searchHistory{};
    std::optional<std::vector<rrt::Node>> path{};

    std::cout << "Starting search..." << std::endl;
    for (int i = 0; i < 100; ++i)
    {
        path = rrt::search(goal, searchTree, context);

        searchHistory.push_back(searchTree);

        if (path.has_value())
        {
            std::cout << "Goal found!" << std::endl;
            break;
        }
    }

    std::cout << "...search done." << std::endl;

    std::cout << "Starting visualization..." << std::endl;
    resetVisualization(m_markerPublisher);
    if (path.has_value())
    {
        visualizePath(path.value(), m_markerPublisher);
    }
    // visualizeMap(map);
    visualizeSearchTree(searchTree, m_markerPublisher);
    std::cout << "...visualization done." << std::endl;
}

} // namespace pet

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    const auto rosNode = std::make_shared<pet::RrtSimulation>();
    rosNode->runRrt();

    rclcpp::spin(rosNode);
    rclcpp::shutdown();
    return 0;
}
