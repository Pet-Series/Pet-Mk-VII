#include "utility/interpolation.hpp"

#include <ugl/lie_group/pose.h>

#include <vector>

namespace pet
{
namespace util
{

std::vector<ugl::lie::Pose> interpolatePath(const ugl::lie::Pose &start,
                                            const ugl::lie::Pose &end, int numberOfPoints)
{
    std::vector<ugl::lie::Pose> path{};
    const double                ratioDelta = 1.0 / (numberOfPoints - 1);
    double                      ratio = 0.0;
    for (int i = 0; i < numberOfPoints; ++i)
    {
        path.push_back(ugl::lie::interpolate(start, end, ratio));
        ratio += ratioDelta;
    }
    return path;
}

} // namespace util
} // namespace pet
