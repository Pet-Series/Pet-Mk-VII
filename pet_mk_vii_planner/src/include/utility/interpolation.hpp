#pragma once

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <vector>

namespace pet
{
namespace util
{

constexpr double interpolate(double a, double b, double ratio)
{
    return a + ratio * (b - a);
}

std::vector<rrt::VehicleState> interpolatePath(const rrt::VehicleState &start,
                                              const rrt::VehicleState &end,
                                              int                     numberOfPoints);

} // namespace util
} // namespace pet
