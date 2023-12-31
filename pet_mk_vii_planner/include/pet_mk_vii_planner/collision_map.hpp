#pragma once

#include <ugl/lie_group/pose.h>
#include <ugl/math/vector.h>

namespace pet::rrt
{

struct BoundingBox
{
    ugl::Vector<2> min;
    ugl::Vector<2> max;
};

using VehicleFootprint = BoundingBox;

class CollisionMap
{
  public:
    bool isInCollision(const ugl::lie::Pose   &pose,
                       const VehicleFootprint &footprint);
};

} // namespace pet::rrt
