#pragma once

#include <ugl/lie_group/pose.h>

#include <vector>

namespace pet::rrt
{

struct ControlInput
{
    double linear_velocity;
    double angular_velocity;
};

struct Node
{
    ugl::lie::Pose m_state;
    ControlInput   m_controlInput;
    Node          *m_parentNode;
};

class Graph
{
  public:
    Graph(const ugl::lie::Pose &startingState);

    // void addRootNode(const Node &rootNode);

    std::vector<Node> getPath() const;

  private:
    std::vector<Node> m_nodes{};
};

} // namespace pet::rrt
