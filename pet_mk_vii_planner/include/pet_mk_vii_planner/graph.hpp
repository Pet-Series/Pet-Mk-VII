#pragma once

#include <ugl/lie_group/pose.h>

#include <functional>
#include <vector>

namespace pet::rrt
{

struct ControlInput
{
    double duration;
    double linear_velocity;
    double angular_velocity;
};

struct Node
{
    int id;
    int parentId;
    std::vector<int> childrenIds;

    ugl::lie::Pose state;
    ControlInput   controlInput;
};

class Graph
{
  public:
    Graph(const ugl::lie::Pose &startingState);

    const Node &addNode(const ugl::lie::Pose &state,
                        const ControlInput &controlInput, const Node &parent);

    const Node &getNode(int id) const;

    Node findClosest(const ugl::lie::Pose &targetPose) const;

    std::vector<Node> getPathFromRoot(const Node &node) const;

    void forEachNode(const std::function<void(const Node &)> &function) const;

  private:
    const Node &storeNode(const Node &node);

  private:
    std::vector<Node> m_nodes;
};

bool isRoot(const Node &node);

} // namespace pet::rrt
