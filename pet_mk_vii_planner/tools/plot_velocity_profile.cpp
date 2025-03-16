#include "velocityProfile.hpp"

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <matplot/matplot.h>

#include <algorithm>
#include <vector>

namespace pet
{

enum class RangeMode
{
    ExcludeEnd,
    IncludeEnd
};

std::vector<double>
range(double start, double end, double step, RangeMode mode = RangeMode::ExcludeEnd)
{
    std::size_t length = static_cast<std::size_t>((end - start) / step);
    if (mode == RangeMode::IncludeEnd)
    {
        ++length;
    }

    std::vector<double> v(length);

    auto generator = [value = start, step]() mutable {
        auto ret = value;
        value += step;
        return ret;
    };
    std::generate(v.begin(), v.end(), generator);

    return v;
}

} // namespace pet

int main()
{
    const pet::rrt::VehicleModel vehicleModel{2.0, 5.0};

    const auto queryPoints = pet::range(0.0, 1.0, 0.01, pet::RangeMode::IncludeEnd);

    const double distance      = 20.0;
    const double startVelocity = 0.0;
    const double endVelocity   = 0.0;

    /// TODO: How to extract total duration it took to drive?
    /// Struct { velocities[], timestamps[], duration }?
    const auto velocitySamples = pet::rrt::computeVelocityProfile(
        queryPoints, distance, startVelocity, endVelocity, vehicleModel);

    matplot::hold(true);

    /// TODO: Scale query points with total duration.
    matplot::plot(queryPoints, velocitySamples)->color("blue");

    matplot::axis(matplot::square);

    matplot::show();
    return 0;
}
