#pragma once

#include "pet_mk_vii_planner/graph.hpp"

#include <rclcpp/publisher.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace pet
{

void visualizePath(
    const std::vector<rrt::Node>                            &path,
    rclcpp::Publisher<visualization_msgs::msg::Marker>      &markerPub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray> &markerArrayPub);

void visualizeSearchTree(const rrt::Graph                                   &tree,
                         rclcpp::Publisher<visualization_msgs::msg::Marker> &markerPub);

void resetVisualization(rclcpp::Publisher<visualization_msgs::msg::Marker> &markerPub);

} // namespace pet
