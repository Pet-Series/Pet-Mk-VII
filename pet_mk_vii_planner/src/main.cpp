#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"

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
        m_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("topic", 10);
    }

    void runRrt();

  private:
    void visualizePath(const std::vector<rrt::Node> &path);

  private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_marker_publisher;
};

void RrtSimulation::runRrt()
{
    const rrt::VehicleFootprint footprint{{-0.02, 0.05}, {0.18, 0.05}};
    const rrt::CollisionMap     map{};
    const rrt::SearchContext    context{10, footprint, map};

    const ugl::lie::Pose startPose = ugl::lie::Pose::Identity();
    const ugl::lie::Pose goalPose{ugl::lie::Rotation::Identity(),
                                  {1.0, 0.0, 0.0}};

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
    if (path.has_value())
    {
        visualizePath(path.value());
    }
    // visualizeMap(map);
    // visualizeSearchHistory(searchHistory);
    std::cout << "...visualization done." << std::endl;
}

void RrtSimulation::visualizePath(const std::vector<rrt::Node> &path)
{
    auto message = visualization_msgs::msg::Marker();
    m_marker_publisher->publish(message);
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
