#pragma once

#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"

#include <ugl/lie_group/pose.h>

#include <optional>
#include <utility>
#include <vector>

namespace pet::rrt
{

struct SearchContext
{
    int m_iterations;

    VehicleFootprint m_vehicleFootprint;
    BoundingBox      m_searchSpace;
    CollisionMap     m_collisionMap;
};

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context);

ugl::lie::Pose sampleState(const Goal &goal, const BoundingBox &searchSpace);

bool shouldSampleFromGoal();

std::optional<std::pair<ControlInput, ugl::lie::Pose>>
tryConnect(const ugl::lie::Pose &start, const ugl::lie::Pose &desiredEnd,
           const CollisionMap &map);

} // namespace pet::rrt
