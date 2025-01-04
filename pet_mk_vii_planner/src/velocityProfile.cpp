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
    const double totalDuration =
        accelerationDuration + cruiseDuration + decelerationDuration;

    // Create vectors with x and y values of velocity trapezoid that we can use to
    // interpolate.
    const std::vector<double> samplePoints = {
        0.0, accelerationDuration, accelerationDuration + cruiseDuration, totalDuration};
    const std::vector<double> sampleValues = {startVel, vehicleModel.maxSpeed,
                                              vehicleModel.maxSpeed, endVel};

    std::vector<double> queryPoints(timeRatios.size());
    std::transform(timeRatios.cbegin(), timeRatios.cend(), queryPoints.begin(),
                   [&](double ratio) { return ratio * totalDuration; });

    velocitySamples = util::interpolate(samplePoints, sampleValues, queryPoints);

    return velocitySamples;
}

} // namespace pet::rrt
