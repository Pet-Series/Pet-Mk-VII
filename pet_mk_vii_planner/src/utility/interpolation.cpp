#include "utility/interpolation.hpp"

#include <ugl/lie_group/pose.h>

#include <cassert>
#include <cmath>
#include <vector>

namespace pet
{
namespace util
{

std::vector<rrt::PoseStamped> interpolatePath(const rrt::PoseStamped &start,
                                              const rrt::PoseStamped &end,
                                              int                     numberOfPoints)
{
    assert(numberOfPoints > 1);
    std::vector<rrt::PoseStamped> path{};

    const double ratioDelta = 1.0 / (numberOfPoints - 1);
    double       ratio      = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        const auto pose      = ugl::lie::interpolate(start.pose, end.pose, ratio);
        const auto timestamp = interpolate(start.timestamp, end.timestamp, ratio);
        path.push_back(rrt::PoseStamped{pose, timestamp});
        ratio += ratioDelta;
    }
    return path;
}

} // namespace util
} // namespace pet
