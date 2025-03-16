#pragma once

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <vector>

namespace pet
{
namespace util
{

/// @brief Linear interpolation between two scalars.
constexpr double interpolate(double a, double b, double ratio)
{
    return a + ratio * (b - a);
}

/// @brief Linearly interpolate values of a sampled function at specific query points.
/// Query points outside the sampled span get the closest sampled value.
/// @param samplePoints the points at which the function has been sampled.
/// @param sampleValues the sampled values of the function.
/// @param queryPoints the query points at which to interpolate.
/// @pre samplePoints and sampleValues must be of equal length.
/// @pre samplePoints and sampleValues must not be empty.
/// @pre samplePoints must be sorted in asceding order.
/// @return Interpolated values at the query points.
std::vector<double> interpolate(const std::vector<double> &samplePoints,
                                const std::vector<double> &sampleValues,
                                const std::vector<double> &queryPoints);

std::vector<rrt::VehicleState> interpolatePath(const rrt::VehicleState &start,
                                               const rrt::VehicleState &end,
                                               int                      numberOfPoints);

} // namespace util
} // namespace pet
