#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrt.hpp"

#include <ugl/lie_group/pose.h>

#include <iostream>
#include <optional>
#include <vector>

namespace pet
{

void main()
{
    const rrt::VehicleFootprint footprint{{-0.02, 0.05}, {0.18, 0.05}};
    const rrt::CollisionMap     map{};
    const rrt::SearchContext    context{10, footprint, map};

    const ugl::lie::Pose startPose = ugl::lie::Pose::Identity();
    const ugl::lie::Pose goalPose{ugl::lie::Rotation::Identity(),
                                  {1.0, 0.0, 0.0}};

    rrt::Graph      searchTree{startPose};
    const rrt::Goal goal{goalPose};

    std::vector<rrt::Graph> searchHistory{};
    std::optional<std::vector<rrt::Node>> path{};

    std::cout << "Starting search..." << std::endl;
    for (int i = 0; i < 100; ++i)
    {
        path = rrt::search(goal, searchTree, context);

        searchHistory.push_back(searchTree);

        if (path.has_value())
        {
            std::cout << "Goal found!" << std::endl;
            break;
        }
    }

    std::cout << "...done." << std::endl;

    // visualiseMap(map);
    // visualisePath(path);
    // visualiseSearchHistory(searchHistory);
}

} // namespace pet

int main() { pet::main(); }
