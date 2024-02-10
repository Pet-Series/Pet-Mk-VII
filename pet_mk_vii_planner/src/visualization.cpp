#include "visualization.hpp"

#include "pet_mk_vii_planner/graph.hpp"
#include "utility/algorithm.hpp"

#include <ugl/lie_group/pose.h>

#include <rclcpp/node.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace pet
{
namespace
{

int getUniqueId()
{
    static int id = 100'000;
    return ++id;
}

geometry_msgs::msg::Point toPointMsg(const ugl::Vector3 &point)
{
    geometry_msgs::msg::Point msg{};
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
    return msg;
}

geometry_msgs::msg::Pose toPoseMsg(const ugl::lie::Pose &pose)
{
    geometry_msgs::msg::Pose msg{};
    msg.position.x = pose.position().x();
    msg.position.y = pose.position().y();
    msg.position.z = pose.position().z();

    const auto quaternion = pose.rotation().to_quaternion();
    msg.orientation.x     = quaternion.x();
    msg.orientation.y     = quaternion.y();
    msg.orientation.z     = quaternion.z();
    msg.orientation.w     = quaternion.w();

    return msg;
}

void appendToLineList(const rrt::Path                        &path,
                      std::vector<geometry_msgs::msg::Point> &points)
{
    util::adjacent_for_each(path.cbegin(), path.cend(),
                            [&points](const auto &start, const auto &end) {
                                points.push_back(toPointMsg(start.pose.position()));
                                points.push_back(toPointMsg(end.pose.position()));
                            });
}

} // namespace

RvizVisualizer::RvizVisualizer(rclcpp::Node &nodeHandle)
{
    m_markerPublisher =
        nodeHandle.create_publisher<visualization_msgs::msg::Marker>("rrt_marker", 10);
    m_markerArrayPublisher =
        nodeHandle.create_publisher<visualization_msgs::msg::MarkerArray>(
            "rrt_marker_array", 10);
}

void RvizVisualizer::visualizePath(const std::vector<rrt::Node> &path)
{
    visualization_msgs::msg::Marker lineList{};
    lineList.header.frame_id = "map";
    lineList.header.stamp    = rclcpp::Time{0};
    lineList.ns              = "rrt";
    lineList.id              = getUniqueId();
    lineList.lifetime        = rclcpp::Duration{0, 0};
    lineList.type            = visualization_msgs::msg::Marker::LINE_LIST;
    lineList.action          = visualization_msgs::msg::Marker::ADD;
    lineList.pose.position.z = 0.01; // Use height to "overlay" in the visualization.
    lineList.scale.x         = 0.05;
    lineList.color.r         = 0.1f;
    lineList.color.g         = 0.8f;
    lineList.color.b         = 0.1f;
    lineList.color.a         = 0.8f;

    std::for_each(path.cbegin(), path.cend(), [&](const rrt::Node &node) {
        if (!rrt::isRoot(node))
        {
            appendToLineList(node.pathFromParent, lineList.points);
        }
    });
    m_markerPublisher->publish(lineList);

    visualization_msgs::msg::MarkerArray arrowArray{};
    visualization_msgs::msg::Marker      arrow{};
    arrow.header.frame_id = "map";
    arrow.header.stamp    = rclcpp::Time{0};
    arrow.ns              = "rrt";
    arrow.id              = getUniqueId();
    arrow.lifetime        = rclcpp::Duration{0, 0};
    arrow.type            = visualization_msgs::msg::Marker::ARROW;
    arrow.action          = visualization_msgs::msg::Marker::ADD;
    arrow.scale.y         = 0.02;
    arrow.scale.z         = 0.02;
    arrow.color.r         = 0.1f;
    arrow.color.g         = 0.9f;
    arrow.color.b         = 0.1f;
    arrow.color.a         = 1.0f;
    for (const auto &node : path)
    {
        for (const auto &pose : node.pathFromParent)
        {
            arrow.pose            = toPoseMsg(pose.pose);
            arrow.scale.x         = pose.velocity / 5.0;
            arrow.pose.position.z = 0.02;
            arrow.id              = getUniqueId();
            arrowArray.markers.push_back(arrow);
        }
    }
    m_markerArrayPublisher->publish(arrowArray);
}

void RvizVisualizer::visualizeSearchTree(const rrt::Graph &tree)
{
    visualization_msgs::msg::Marker lineList{};

    lineList.header.frame_id = "map";
    lineList.header.stamp    = rclcpp::Time{0};
    lineList.ns              = "rrt";
    lineList.id              = getUniqueId();
    lineList.lifetime        = rclcpp::Duration{0, 0};
    lineList.type            = visualization_msgs::msg::Marker::LINE_LIST;
    lineList.action          = visualization_msgs::msg::Marker::ADD;
    lineList.scale.x         = 0.02;
    lineList.color.r         = 0.3f;
    lineList.color.g         = 0.6f;
    lineList.color.b         = 1.0f;
    lineList.color.a         = 0.6f;

    tree.forEachNode([&](const rrt::Node &node) {
        if (!rrt::isRoot(node))
        {
            appendToLineList(node.pathFromParent, lineList.points);
        }
    });
    m_markerPublisher->publish(lineList);

    visualization_msgs::msg::MarkerArray arrowArray{};
    visualization_msgs::msg::Marker      arrow{};
    arrow.header.frame_id = "map";
    arrow.header.stamp    = rclcpp::Time{0};
    arrow.ns              = "rrt";
    arrow.id              = getUniqueId();
    arrow.lifetime        = rclcpp::Duration{0, 0};
    arrow.type            = visualization_msgs::msg::Marker::ARROW;
    arrow.action          = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x         = 0.2;
    arrow.scale.y         = 0.02;
    arrow.scale.z         = 0.02;
    arrow.color.r         = 0.3f;
    arrow.color.g         = 0.6f;
    arrow.color.b         = 1.0f;
    arrow.color.a         = 0.6f;
    tree.forEachNode([&](const rrt::Node &node) {
        arrow.pose            = toPoseMsg(node.state.pose);
        arrow.pose.position.z = 0.01;
        arrow.id              = getUniqueId();
        arrowArray.markers.push_back(arrow);
    });
    m_markerArrayPublisher->publish(arrowArray);
}

void RvizVisualizer::resetVisualization()
{
    visualization_msgs::msg::Marker marker{};
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    m_markerPublisher->publish(marker);

    visualization_msgs::msg::MarkerArray markerArray{};
    markerArray.markers.push_back(marker);
    m_markerArrayPublisher->publish(markerArray);
}

} // namespace pet
