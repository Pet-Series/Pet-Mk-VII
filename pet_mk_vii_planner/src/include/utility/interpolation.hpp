#pragma once

#include "pet_mk_vii_planner/graph.hpp"

#include <vector>

namespace pet
{
namespace util
{

constexpr double interpolate(double a, double b, double ratio)
{
    return a + ratio * (b - a);
}

std::vector<rrt::PoseStamped> interpolatePath(const rrt::PoseStamped &start,
                                              const rrt::PoseStamped &end,
                                              int                     numberOfPoints);

} // namespace util
} // namespace pet
