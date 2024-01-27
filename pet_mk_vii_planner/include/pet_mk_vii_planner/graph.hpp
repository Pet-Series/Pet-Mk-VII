#pragma once

#include <ugl/lie_group/pose.h>

#include <functional>
#include <vector>

namespace pet::rrt
{

struct VehicleState
{
    ugl::lie::Pose pose;
    double         linearVelocity;
    double         angularVelocity;
};

struct PoseStamped
{
    ugl::lie::Pose pose;
    double         timestamp;
};

using Path = std::vector<PoseStamped>;

struct Node
{
    int              id;
    int              parentId;
    std::vector<int> childrenIds;

    VehicleState state;
    Path         pathFromParent;
};

/// TODO: Define struct 'Payload' and make Graph only interact with that?

class Graph
{
  public:
    Graph(const VehicleState &startingState);

    const Node &addNode(const VehicleState &state, const Path &pathFromParent,
                        const Node &parent);

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
