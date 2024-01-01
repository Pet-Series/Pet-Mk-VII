#include "visualization.hpp"

#include "pet_mk_vii_planner/graph.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace pet
{

void visualizePath(
    const std::vector<rrt::Node>                                  &path,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &markerPub)
{
    visualization_msgs::msg::Marker lineStrip{};

    lineStrip.header.frame_id = "map";
    lineStrip.header.stamp = rclcpp::Time{0};
    lineStrip.ns = "rrt";
    lineStrip.id = 1;
    lineStrip.lifetime = rclcpp::Duration{0, 0};
    lineStrip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lineStrip.action = visualization_msgs::msg::Marker::ADD;
    lineStrip.scale.x = 0.05;
    lineStrip.color.g = 1.0;
    lineStrip.color.a = 1.0;

    for (const auto &node : path)
    {
        geometry_msgs::msg::Point point{};
        point.x = node.state.position().x();
        point.y = node.state.position().y();
        point.z = 0.01;
        lineStrip.points.push_back(point);
    }

    markerPub->publish(lineStrip);
}

void visualizeSearchTree(
    const rrt::Graph                                              &tree,
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &markerPub)
{
    visualization_msgs::msg::Marker marker{};

    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Time{0};
    marker.ns = "rrt";
    marker.id = 101;
    marker.lifetime = rclcpp::Duration{0, 0};
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.02;
    marker.color.b = 1.0;
    marker.color.a = 0.8;

    tree.forEachNode([&marker, &tree](const rrt::Node &node) mutable {
        if (!rrt::isRoot(node))
        {
            const auto &parent = tree.getNode(node.parentId);

            geometry_msgs::msg::Point parentPoint{};
            parentPoint.x = parent.state.position().x();
            parentPoint.y = parent.state.position().y();
            parentPoint.z = 0.0;
            marker.points.push_back(parentPoint);

            geometry_msgs::msg::Point childPoint{};
            childPoint.x = node.state.position().x();
            childPoint.y = node.state.position().y();
            childPoint.z = 0.0;
            marker.points.push_back(childPoint);
        }
    });

    markerPub->publish(marker);
}

void resetVisualization(
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr &markerPub)
{
    visualization_msgs::msg::Marker marker{};
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerPub->publish(marker);
}

} // namespace pet
