#pragma once

#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"

#include <optional>
#include <utility>
#include <vector>

namespace pet::rrt
{

struct VehicleModel
{
    double maxSpeed     = 0.5;
    double maxCurvature = 2.0;
    // double maxSpeed     = 10.0;
    // double maxCurvature = 20.0;
};

using SteerFunction = std::optional<std::pair<VehicleState, Path>> (*)(
    const VehicleState &start, const VehicleState &desiredEnd,
    const VehicleModel &vehicleModel);

struct SearchContext
{
    int maxIterations;

    VehicleModel     vehicleModel;
    VehicleFootprint vehicleFootprint;
    BoundingBox      searchSpace;
    CollisionMap     collisionMap;

    SteerFunction steerFunction;
};

std::optional<std::vector<Node>> search(const Goal &goal, Graph &tree,
                                        const SearchContext &context);

VehicleState sampleState(const Goal &goal, const VehicleModel &vehicleModel,
                         const BoundingBox &searchSpace);

bool shouldSampleFromGoal();

std::optional<std::pair<VehicleState, Path>>
steerBezier(const VehicleState &start, const VehicleState &desiredEnd,
            const VehicleModel &vehicleModel);

std::optional<std::pair<VehicleState, Path>> steerCtrv(const VehicleState &start,
                                                       const VehicleState &desiredEnd,
                                                       const VehicleModel &vehicleModel);

} // namespace pet::rrt
