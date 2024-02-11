#pragma once

#include <ugl/lie_group/pose.h>

#include <vector>

namespace pet::rrt
{ 

struct VehicleModel
{
    double maxSpeed     = 0.5;
    double maxCurvature = 2.0;
};

struct VehicleState
{
    ugl::lie::Pose pose;
    double         velocity;
};

struct PoseStamped
{
    ugl::lie::Pose pose;
    double         velocity;
    double         timestamp;
};

using Path = std::vector<PoseStamped>;

} // namespace pet::rrt