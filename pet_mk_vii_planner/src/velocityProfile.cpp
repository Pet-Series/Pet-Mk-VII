#include "velocityProfile.hpp"

#include <algorithm>
#include <iterator>
#include <vector>

namespace pet::rrt
{

std::vector<double> computeVelocityProfile(const std::vector<double> &sampleRatios,
                                           double totalDistance, double startVel,
                                           double              endVel,
                                           const VehicleModel &vehicleModel)
{
    std::vector<double> velocitySamples{};
    velocitySamples.reserve(sampleRatios.size());

    std::transform(sampleRatios.cbegin(), sampleRatios.cend(),
                   std::back_inserter(velocitySamples),
                   [](double ratio) { return ratio; });

    return velocitySamples;
}

} // namespace pet::rrt
