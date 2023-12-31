#include "pet_mk_vii_planner/rrt.hpp"

namespace pet::rrt
{

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context)
{
    const auto sampledState = goal.sampleState();

    const Node &parentNode = tree.findClosest(sampledState);

    /// TODO: Add steering function.
    ControlInput controlInput{};

    const Node &newNode = tree.addNode(sampledState, controlInput, parentNode);

    /// TODO: Add check if result from steering is in goal.
    const bool goalReached = true;
    if (goalReached)
    {
        return tree.getPathFromRoot(newNode);
    }
    else
    {
        return {};
    }
}

} // namespace pet::rrt
