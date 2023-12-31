#include "pet_mk_vii_planner/rrt.hpp"

namespace pet::rrt
{

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context)
{
    for (int i = 0; i < context.m_iterations; ++i)
    {
        const auto sampledState = goal.sampleState();

        const Node &parentNode = tree.findClosest(sampledState);

        /// TODO: Add steering function.
        ControlInput controlInput{};

        const Node &newNode = tree.addNode(sampledState, controlInput, parentNode);

        if (goal.isReached(sampledState))
        {
            return tree.getPathFromRoot(newNode);
        }
    }
    return {};
}

} // namespace pet::rrt
