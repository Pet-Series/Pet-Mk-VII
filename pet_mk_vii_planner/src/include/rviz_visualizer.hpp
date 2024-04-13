#pragma once

#include "pet_mk_vii_planner/graph.hpp"

#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>

namespace pet
{

class RvizVisualizer
{
  public:
    explicit RvizVisualizer(rclcpp::Node &nodeHandle);

    void visualizePath(const std::vector<rrt::Node> &path);

    void visualizeSearchTree(const rrt::Graph &tree);

    void resetVisualization();

  private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_markerPublisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        m_markerArrayPublisher;
};

} // namespace pet
