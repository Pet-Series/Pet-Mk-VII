#pragma once

#include <ugl/lie_group/pose.h>

#include <vector>

namespace pet
{
namespace util
{

std::vector<ugl::lie::Pose> interpolatePath(const ugl::lie::Pose &start,
                                            const ugl::lie::Pose &end,
                                            int                   numberOfPoints);

} // namespace util
} // namespace pet
