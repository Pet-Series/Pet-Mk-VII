#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"

#include <ugl/lie_group/pose.h>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <iostream>
#include <optional>
#include <vector>

namespace pet
{

using namespace std::chrono_literals;

class RrtSimulation : public rclcpp::Node
{
  public:
    RrtSimulation() : Node("rrt_simulation")
    {
        m_marker_publisher =
            this->create_publisher<visualization_msgs::msg::Marker>("rrt_found_path", 10);
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
    const rrt::BoundingBox      searchSpace{{-10.0, -10.0}, {10.0, 10.0}};
    const rrt::CollisionMap     map{};
    const rrt::SearchContext    context{10, footprint, searchSpace, map};

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
    auto marker = visualization_msgs::msg::Marker();

    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time{0};
    marker.ns = "rrt";
    marker.id = 0;
    marker.lifetime = rclcpp::Duration{0, 0};
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    for (const auto &node : path)
    {
        geometry_msgs::msg::Point point{};
        point.x = node.m_state.position().x();
        point.y = node.m_state.position().y();
        marker.points.push_back(point);
    }

    m_marker_publisher->publish(marker);
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
