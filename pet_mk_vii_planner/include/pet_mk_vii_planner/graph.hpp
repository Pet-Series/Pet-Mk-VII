#pragma once

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <functional>
#include <vector>

namespace pet::rrt
{

struct NodeId
{
    int bucketIndex;
    int internalIndex;
};

struct Node
{
    NodeId              id;
    NodeId              parentId;
    std::vector<NodeId> childrenIds;

    VehicleState state;
    Path         pathFromParent;
};

/// TODO: Define struct 'Payload' and make Graph only interact with that?

class Graph
{
  public:
    Graph(const VehicleState &startingState, const BoundingBox &boundingBox);

    const Node &addNode(const VehicleState &state, const Path &pathFromParent,
                        const Node &parent);

    Node       &getNode(const NodeId &id);
    const Node &getNode(const NodeId &id) const;

    Node findClosest(const VehicleState &targetState) const;

    Node sampleClose(const VehicleState &targetState) const;

    std::vector<Node> getPathFromRoot(const Node &node) const;

    void forEachNode(const std::function<void(const Node &)> &function) const;

  private:
    const Node &storeNode(const Node &node);

    int findBucketIndex(const Node &node) const;

  private:
    std::vector<std::vector<Node>> m_buckets;

    BoundingBox m_boundingBox;
    int         m_numSidesX;
    int         m_numSidesY;

    static constexpr double kBucketSize = 1.0;
};

bool isRoot(const Node &node);

} // namespace pet::rrt
