#include "visualization.hpp"

#include <ugl/lie_group/pose.h>

#include "pet_mk_vii_planner/graph.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace pet
{
namespace
{

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
    visualization_msgs::msg::Marker lineStrip{};
    lineStrip.header.frame_id = "map";
    lineStrip.header.stamp = rclcpp::Time{0};
    lineStrip.ns = "rrt";
    lineStrip.id = 1;
    lineStrip.lifetime = rclcpp::Duration{0, 0};
    lineStrip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    lineStrip.action = visualization_msgs::msg::Marker::ADD;
    lineStrip.scale.x = 0.05;
    lineStrip.color.r = 0.1;
    lineStrip.color.g = 0.8;
    lineStrip.color.b = 0.1;
    lineStrip.color.a = 0.8;
    for (const auto &node : path)
    {
        geometry_msgs::msg::Point point{};
        point.x = node.state.position().x();
        point.y = node.state.position().y();
        point.z = 0.01;
        lineStrip.points.push_back(point);
    }
    markerPub.publish(lineStrip);

    visualization_msgs::msg::MarkerArray arrowArray{};
    visualization_msgs::msg::Marker      arrow{};
    arrow.header.frame_id = "map";
    arrow.header.stamp = rclcpp::Time{0};
    arrow.ns = "rrt";
    arrow.id = 2;
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

void visualizeSearchTree(const rrt::Graph                                   &tree,
                         rclcpp::Publisher<visualization_msgs::msg::Marker> &markerPub)
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
    markerPub.publish(marker);
}

void resetVisualization(rclcpp::Publisher<visualization_msgs::msg::Marker> &markerPub)
{
    visualization_msgs::msg::Marker marker{};
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markerPub.publish(marker);
}

} // namespace pet
