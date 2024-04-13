#pragma once

#include <pet_mk_vii_planner/rrtDefinitions.hpp>

#include <ugl/lie_group/pose.h>
#include <ugl/math/vector.h>

namespace pet::rrt
{

using VehicleFootprint = BoundingBox;

class CollisionMap
{
  public:
    bool isInCollision(const ugl::lie::Pose &pose, const VehicleFootprint &footprint);
};

} // namespace pet::rrt
