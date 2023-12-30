#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"

#include <ugl/lie_group/pose.h>

#include <iostream>
#include <vector>

namespace pet
{

void main()
{
    const rrt::VehicleFootprint footprint{{-0.02, 0.05}, {0.18, 0.05}};
    const rrt::SearchContext    context{10, footprint, rrt::CollisionMap{}};

    const ugl::lie::Pose startPose = ugl::lie::Pose::Identity();
    const ugl::lie::Pose goalPose{ugl::lie::Rotation::Identity(),
                                  {1.0, 0.0, 0.0}};

    rrt::Graph      searchTree{startPose};
    const rrt::Goal goal{goalPose};

    std::vector<rrt::Graph> searchHistory{};
    for (int i = 0; i < 100; ++i)
    {
        const bool goalFound = rrt::search(goal, searchTree, context);

        searchHistory.push_back(searchTree);

        if (goalFound)
        {
            /// TODO: Celebrate!
            break;
        }
    }

    std::cout << "Done!" << std::endl;

    // visualisePath(searchTree.getPath());
    // visualiseSearchHistory(searchHistory);
}

} // namespace pet

int main() { pet::main(); }
