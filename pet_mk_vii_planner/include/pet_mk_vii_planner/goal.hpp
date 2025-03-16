#pragma once

#include <pet_mk_vii_planner/graph.hpp>

#include <ugl/lie_group/pose.h>

namespace pet::rrt
{

class Goal
{
  public:
    explicit Goal(const ugl::lie::Pose &targetPose) : m_targetPose(targetPose) {};

    VehicleState sampleState() const;

    bool isReached(const VehicleState &state) const;

  private:
    ugl::lie::Pose m_targetPose;
    double         m_targetSpeed = 0.0;

    static constexpr double m_positionTolerance = 0.1;
    static constexpr double m_headingTolerance  = 0.5;
    static constexpr double m_speedTolerance    = 0.01;
};

} // namespace pet::rrt
