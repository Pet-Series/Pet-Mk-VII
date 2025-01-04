#pragma once

#include <ugl/lie_group/pose.h>

#include <vector>

namespace pet::rrt
{

struct VehicleModel
{
    double maxSpeed        = 0.5;
    double maxCurvature    = 2.0;
    double maxAcceleration = 0.5; /// TODO: Read from parameter file.
    double maxDeceleration = 0.5; /// TODO: Read from parameter file.
};

struct VehicleState
{
    ugl::lie::Pose pose;
    double         velocity;
    double         timestamp = 0.0;
};

using Path = std::vector<VehicleState>;

struct BoundingBox
{
    ugl::Vector<2> min;
    ugl::Vector<2> max;
};

} // namespace pet::rrt
