#include "velocityProfile.hpp"

#include "utility/interpolation.hpp"

#include <algorithm>
#include <iterator>
#include <vector>

namespace pet::rrt
{
namespace
{

template <typename T> constexpr T square(T x) { return x * x; }

} // namespace

std::vector<double> computeVelocityProfile(const std::vector<double> &timeRatios,
                                           double totalDistance, double startVel, double endVel,
                                           const VehicleModel &vehicleModel)
{
    /// TODO: Handle negative start- and end- velocities.
    std::vector<double> velocitySamples{};
    velocitySamples.reserve(timeRatios.size());

    if (std::abs(startVel) > vehicleModel.maxSpeed || std::abs(endVel) > vehicleModel.maxSpeed)
    {
        // In case of mutually inconsistent velocity constraints, return an empty list and
        // let the caller handle the situation.
        return velocitySamples;
    }

    const double accelerationDuration =
        std::abs(vehicleModel.maxSpeed - startVel) / vehicleModel.maxAcceleration;
    const double decelerationDuration =
        std::abs(vehicleModel.maxSpeed - endVel) / vehicleModel.maxDeceleration;

    const double accelerationDistance =
        startVel * accelerationDuration +
        vehicleModel.maxAcceleration * square(accelerationDuration) / 2.0;
    const double decelerationDistance =
        endVel * decelerationDuration +
        vehicleModel.maxDeceleration * square(decelerationDuration) / 2.0;
    const double cruiseDistance = totalDistance - (accelerationDistance + decelerationDistance);

    const double cruiseDuration = cruiseDistance / vehicleModel.maxSpeed;
    const double totalDuration  = accelerationDuration + decelerationDuration + cruiseDuration;

    /// TODO: Implement in terms of matlab's interp1(), i.e. f(t) = interp1(x, f(x), t).
    auto interpolateVelocity = [&](double ratio) {
        const double scaledRatio = ratio * totalDuration;
        if (scaledRatio < accelerationDuration)
        {
            // Is in acceleration phase.
            return util::interpolate(startVel, vehicleModel.maxSpeed,
                                     scaledRatio / accelerationDuration);
        }
        else if (scaledRatio > (totalDuration - decelerationDuration))
        {
            // Is in deceleration phase.
            double decelerationRatio =
                scaledRatio - (totalDuration - decelerationDuration);
            return util::interpolate(vehicleModel.maxSpeed, endVel,
                                     decelerationRatio / decelerationDuration);
        }
        else
        {
            // Is in cruising phase.
            return vehicleModel.maxSpeed;
        }
    };

    std::transform(timeRatios.cbegin(), timeRatios.cend(), std::back_inserter(velocitySamples),
                   [&](double ratio) { return interpolateVelocity(ratio); });

    return velocitySamples;
}

} // namespace pet::rrt
