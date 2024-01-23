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

struct VehicleModel
{
    // double maxSpeed = 0.5;
    // double maxCurvature = 2.0;
    double maxSpeed = 10.0;
    double maxCurvature = 20.0;
};

struct SearchContext
{
    int maxIterations;

    VehicleModel     vehicleModel;
    VehicleFootprint vehicleFootprint;
    BoundingBox      searchSpace;
    CollisionMap     collisionMap;
};

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context);

ugl::lie::Pose sampleState(const Goal &goal, const BoundingBox &searchSpace);

bool shouldSampleFromGoal();

std::optional<std::pair<ControlInput, ugl::lie::Pose>>
tryConnect(const ugl::lie::Pose &start, const ugl::lie::Pose &desiredEnd,
           const SearchContext &context);

} // namespace pet::rrt
