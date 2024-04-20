#pragma once

#include "pet_mk_vii_planner/collision_map.hpp"
#include "pet_mk_vii_planner/goal.hpp"
#include "pet_mk_vii_planner/graph.hpp"
#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <cstdint>
#include <optional>
#include <utility>

namespace pet::rrt
{

using SteerFunction = std::optional<std::pair<VehicleState, Path>> (*)(
    const VehicleState &start, const VehicleState &desiredEnd,
    const VehicleModel &vehicleModel);

struct SearchContext
{
    std::int64_t maxIterations;

    VehicleModel     vehicleModel;
    VehicleFootprint vehicleFootprint;
    BoundingBox      searchSpace;
    CollisionMap     collisionMap;

    SteerFunction steerFunction;
};

struct SearchDiagnostics
{
    int totalIterations  = 0;
    int totalConnections = 0;

    SearchDiagnostics &operator+=(const SearchDiagnostics &other)
    {
        totalIterations += other.totalIterations;
        totalConnections += other.totalConnections;
        return *this;
    }
};

struct SearchResult
{
    std::optional<std::vector<Node>> path{};
    SearchDiagnostics                diag{};
};

SearchResult search(const Goal &goal, Graph &tree, const SearchContext &context);

VehicleState sampleState(const Goal &goal, const VehicleModel &vehicleModel,
                         const BoundingBox &searchSpace);

bool shouldSampleFromGoal();

} // namespace pet::rrt
