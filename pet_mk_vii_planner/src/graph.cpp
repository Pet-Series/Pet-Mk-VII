#include "pet_mk_vii_planner/graph.hpp"

namespace pet::rrt
{

Graph::Graph(const ugl::lie::Pose &startingState)
{
    Node rootNode{};
    rootNode.m_state = startingState;
    rootNode.m_parentNode = nullptr;

    m_nodes.push_back(rootNode);
}

} // namespace pet::rrt
