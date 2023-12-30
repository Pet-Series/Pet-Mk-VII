#pragma once

#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"

namespace pet::rrt
{

struct SearchContext
{
    int m_iterations;

    VehicleFootprint m_vehicleFootprint;
    CollisionMap     m_collisionMap;
};

bool search(const Goal &goalPose, Graph &searchTree,
            const SearchContext &context);

} // namespace pet::rrt
