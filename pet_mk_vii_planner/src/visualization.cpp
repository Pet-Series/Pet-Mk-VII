#include "visualization.hpp"

#include "pet_mk_vii_planner/graph.hpp"
#include "utility/algorithm.hpp"

#include <ugl/lie_group/pose.h>

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

std::vector<ugl::lie::Pose> interpolatePath(const ugl::lie::Pose &start,
                                            const ugl::lie::Pose &end, int numberOfPoints)
{
    std::vector<ugl::lie::Pose> path{};
    const double                ratioDelta = 1.0 / (numberOfPoints - 1);
    double                      ratio = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        path.push_back(ugl::lie::interpolate(start, end, ratio));
        ratio += ratioDelta;
    }
    return path;
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
    msg.orientation.x = quaternion.x();
    msg.orientation.y = quaternion.y();
    msg.orientation.z = quaternion.z();
    msg.orientation.w = quaternion.w();

    return msg;
}

} // namespace

void visualizePath(
    const std::vector<rrt::Node>                            &path,
    rclcpp::Publisher<visualization_msgs::msg::Marker>      &markerPub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray> &markerArrayPub)
{
    visualization_msgs::msg::Marker lineList{};
    lineList.header.frame_id = "map";
    lineList.header.stamp = rclcpp::Time{0};
    lineList.ns = "rrt";
    lineList.id = getUniqueId();
    lineList.lifetime = rclcpp::Duration{0, 0};
    lineList.type = visualization_msgs::msg::Marker::LINE_LIST;
    lineList.action = visualization_msgs::msg::Marker::ADD;
    lineList.pose.position.z = 0.01; // Use height to "overlay" in the visualization.
    lineList.scale.x = 0.05;
    lineList.color.r = 0.1;
    lineList.color.g = 0.8;
    lineList.color.b = 0.1;
    lineList.color.a = 0.8;

    util::adjacent_for_each(
        path.cbegin(), path.cend(),
        [&lineList](const rrt::Node &parent, const rrt::Node &child) {
            const auto path = interpolatePath(parent.state, child.state, 5);
            util::adjacent_for_each(
                path.cbegin(), path.cend(),
                [&lineList](const auto &start, const auto &end) {
                    lineList.points.push_back(toPointMsg(start.position()));
                    lineList.points.push_back(toPointMsg(end.position()));
                });
        });
    markerPub.publish(lineList);

    visualization_msgs::msg::MarkerArray arrowArray{};
    visualization_msgs::msg::Marker      arrow{};
    arrow.header.frame_id = "map";
    arrow.header.stamp = rclcpp::Time{0};
    arrow.ns = "rrt";
    arrow.id = getUniqueId();
    arrow.lifetime = rclcpp::Duration{0, 0};
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.2;
    arrow.scale.y = 0.02;
    arrow.scale.z = 0.02;
    arrow.color.r = 0.1;
    arrow.color.g = 0.9;
    arrow.color.b = 0.1;
    arrow.color.a = 1.0;
    for (const auto &node : path)
    {
        arrow.pose = toPoseMsg(node.state);
        arrow.pose.position.z = 0.02;
        ++arrow.id;
        arrowArray.markers.push_back(arrow);
    }
    markerArrayPub.publish(arrowArray);
}

void visualizeSearchTree(
    const rrt::Graph &tree, rclcpp::Publisher<visualization_msgs::msg::Marker> &markerPub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray> &markerArrayPub)
{
    visualization_msgs::msg::Marker lineList{};

    lineList.header.frame_id = "map";
    lineList.header.stamp = rclcpp::Time{0};
    lineList.ns = "rrt";
    lineList.id = getUniqueId();
    lineList.lifetime = rclcpp::Duration{0, 0};
    lineList.type = visualization_msgs::msg::Marker::LINE_LIST;
    lineList.action = visualization_msgs::msg::Marker::ADD;
    lineList.scale.x = 0.02;
    lineList.color.r = 0.3;
    lineList.color.g = 0.6;
    lineList.color.b = 1.0;
    lineList.color.a = 0.6;

    tree.forEachNode([&](const rrt::Node &node) mutable {
        if (!rrt::isRoot(node))
        {
            const auto &parent = tree.getNode(node.parentId);
            const auto  path = interpolatePath(parent.state, node.state, 5);
            util::adjacent_for_each(
                path.cbegin(), path.cend(),
                [&lineList](const auto &start, const auto &end) {
                    lineList.points.push_back(toPointMsg(start.position()));
                    lineList.points.push_back(toPointMsg(end.position()));
                });
        }
    });
    markerPub.publish(lineList);

    visualization_msgs::msg::MarkerArray arrowArray{};
    visualization_msgs::msg::Marker      arrow{};
    arrow.header.frame_id = "map";
    arrow.header.stamp = rclcpp::Time{0};
    arrow.ns = "rrt";
    arrow.id = getUniqueId();
    arrow.lifetime = rclcpp::Duration{0, 0};
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = visualization_msgs::msg::Marker::ADD;
    arrow.scale.x = 0.2;
    arrow.scale.y = 0.02;
    arrow.scale.z = 0.02;
    arrow.color.r = 0.3;
    arrow.color.g = 0.6;
    arrow.color.b = 1.0;
    arrow.color.a = 0.6;
    tree.forEachNode([&](const rrt::Node &node) mutable {
        arrow.pose = toPoseMsg(node.state);
        arrow.pose.position.z = 0.01;
        ++arrow.id;
        arrowArray.markers.push_back(arrow);
    });
    markerArrayPub.publish(arrowArray);
}

void resetVisualization(rclcpp::Publisher<visualization_msgs::msg::Marker> &markerPub)
{
    visualization_msgs::msg::Marker marker{};
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerPub.publish(marker);
}

} // namespace pet
