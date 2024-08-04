#include "velocityProfile.hpp"

#include "utility/interpolation.hpp"

#include <algorithm>
#include <iterator>
#include <vector>

namespace pet::rrt
{

std::vector<double> computeVelocityProfile(const std::vector<double> &timeRatios,
                                           double totalDistance, double startVel,
                                           double              endVel,
                                           const VehicleModel &vehicleModel)
{
    std::vector<double> velocitySamples{};

    if (std::abs(startVel) > vehicleModel.maxSpeed ||
        std::abs(endVel) > vehicleModel.maxSpeed)
    {
        // In case of mutually inconsistent velocity constraints, return an empty list and
        // let the caller handle the situation.
        return velocitySamples;
    }

    velocitySamples.reserve(timeRatios.size());

    std::transform(
        timeRatios.cbegin(), timeRatios.cend(), std::back_inserter(velocitySamples),
        [&](double ratio) { return util::interpolate(startVel, endVel, ratio); });

    return velocitySamples;
}

} // namespace pet::rrt
