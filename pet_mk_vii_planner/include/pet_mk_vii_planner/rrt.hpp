#pragma once

#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"

#include <optional>
#include <vector>

namespace pet::rrt
{

struct SearchContext
{
    int m_iterations;

    VehicleFootprint m_vehicleFootprint;
    CollisionMap     m_collisionMap;
};

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context);

} // namespace pet::rrt
