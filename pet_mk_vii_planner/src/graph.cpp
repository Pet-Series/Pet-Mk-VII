#include "pet_mk_vii_planner/graph.hpp"

#include <algorithm>
#include <limits>

namespace pet::rrt
{

Graph::Graph(const ugl::lie::Pose &startingState)
{
    Node rootNode{};
    rootNode.parentId = -1;
    rootNode.state = startingState;
    storeNode(rootNode);
}

const Node &Graph::addNode(const ugl::lie::Pose &state,
                           const ControlInput &controlInput, const Node &parent)
{
    Node node{};
    node.parentId = parent.id;
    node.state = state;
    node.controlInput = controlInput;
    return storeNode(node);
}

Node Graph::findClosest(const ugl::lie::Pose &targetPose) const
{
    /// TODO: Implement some sort of tree- and/or bucket- system to improve performance.
    Node   nearestNode = m_nodes.back();
    double minDistance = std::numeric_limits<double>::infinity();
    for (const auto &node : m_nodes)
    {
        const double distance = ugl::lie::ominus(node.state, targetPose).squaredNorm();
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
    while (currentNode.parentId > -1)
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
    storedNode.id = m_nodes.size() - 1;
    if (storedNode.parentId != -1)
    {
        m_nodes[storedNode.parentId].childrenIds.push_back(storedNode.id);
    }
    return storedNode;
}

} // namespace pet::rrt
