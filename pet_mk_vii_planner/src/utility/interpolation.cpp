#include "utility/interpolation.hpp"

#include <ugl/lie_group/pose.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

namespace pet
{
namespace util
{

std::vector<double> interpolate(const std::vector<double> &samplePoints,
                                const std::vector<double> &sampleValues,
                                const std::vector<double> &queryPoints)
{
    assert(!samplePoints.empty());
    assert(!sampleValues.empty());
    assert(samplePoints.size() == sampleValues.size());
    assert(std::is_sorted(samplePoints.cbegin(), samplePoints.cend()));

    auto computeQueryValue = [&](double queryPoint) {
        if (queryPoint <= samplePoints.front())
        {
            return sampleValues.front();
        }
        if (queryPoint >= samplePoints.back())
        {
            return sampleValues.back();
        }

        // Find the pair of samples adjacent to the query point and run linear
        // interpolation.
        for (std::size_t i = 0; i < samplePoints.size() || i < sampleValues.size(); ++i)
        {
            if (queryPoint < samplePoints[i])
            {
                if (std::abs(samplePoints[i] - samplePoints[i - 1]) > 1e-8)
                {
                    const double ratio = (queryPoint - samplePoints[i - 1]) /
                                         std::abs(samplePoints[i] - samplePoints[i - 1]);
                    return interpolate(sampleValues[i - 1], sampleValues[i], ratio);
                }
                else
                {
                    // Return the average as a "best-effort" in case the samples are too
                    // close.
                    return (sampleValues[i - 1] + sampleValues[i]) / 2.0;
                }
            }
        }

        throw "This line should never be reached. Shame on me!";
    };

    std::vector<double> queryValues(queryPoints.size());
    std::transform(queryPoints.cbegin(), queryPoints.cend(), queryValues.begin(),
                   computeQueryValue);
    return queryValues;
}

std::vector<rrt::VehicleState> interpolatePath(const rrt::VehicleState &start,
                                              const rrt::VehicleState &end,
                                              int                     numberOfPoints)
{
    assert(numberOfPoints > 1);
    std::vector<rrt::VehicleState> path{};

    const double ratioDelta = 1.0 / (numberOfPoints - 1);
    double       ratio      = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        const auto pose      = ugl::lie::interpolate(start.pose, end.pose, ratio);
        const auto velocity  = interpolate(start.velocity, end.velocity, ratio);
        const auto timestamp = interpolate(start.timestamp, end.timestamp, ratio);
        path.push_back(rrt::VehicleState{pose, velocity, timestamp});
        ratio += ratioDelta;
    }
    return path;
}

} // namespace util
} // namespace pet
