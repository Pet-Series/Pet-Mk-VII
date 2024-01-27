#include "pet_mk_vii_planner/graph.hpp"

#include <algorithm>
#include <limits>

namespace pet::rrt
{

Graph::Graph(const VehicleState &startingState)
{
    Node rootNode{};
    rootNode.parentId       = -1;
    rootNode.state          = startingState;
    rootNode.pathFromParent = {};
    storeNode(rootNode);
}

const Node &Graph::addNode(const VehicleState &state, const Path &pathFromParent,
                           const Node &parent)
{
    Node node{};
    node.parentId       = parent.id;
    node.state          = state;
    node.pathFromParent = pathFromParent;
    return storeNode(node);
}

const Node &Graph::getNode(int id) const { return m_nodes.at(id); }

Node Graph::findClosest(const ugl::lie::Pose &targetPose) const
{
    /// TODO: Create overload of findClosest that takes a binary distance function.
    /// TODO: Implement some sort of tree- and/or bucket- system to improve performance.
    Node   nearestNode = m_nodes.back();
    double minDistance = std::numeric_limits<double>::infinity();
    for (const auto &node : m_nodes)
    {
        const double distance =
            ugl::lie::ominus(node.state.pose, targetPose).squaredNorm();
        if (distance < minDistance)
        {
            minDistance = distance;
            nearestNode = node;
        }
    }
    return nearestNode;
}

std::vector<Node> Graph::getPathFromRoot(const Node &node) const
{
    Node              currentNode = node;
    std::vector<Node> path{currentNode};
    while (!isRoot(currentNode))
    {
        currentNode = m_nodes[currentNode.parentId];
        path.push_back(currentNode);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

const Node &Graph::storeNode(const Node &node)
{
    m_nodes.push_back(node);
    Node &storedNode = m_nodes.back();
    storedNode.id    = m_nodes.size() - 1;
    if (!isRoot(storedNode))
    {
        m_nodes[storedNode.parentId].childrenIds.push_back(storedNode.id);
    }
    return storedNode;
}

void Graph::forEachNode(const std::function<void(const Node &)> &function) const
{
    std::for_each(m_nodes.begin(), m_nodes.end(), function);
}

bool isRoot(const Node &node) { return node.parentId == -1; }

} // namespace pet::rrt
