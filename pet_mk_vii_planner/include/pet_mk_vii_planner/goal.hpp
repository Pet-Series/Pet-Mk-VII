#pragma once

#include <ugl/lie_group/pose.h>

namespace pet::rrt
{

class Goal
{
  public:
    explicit Goal(const ugl::lie::Pose &goalPose) : m_goalPose(goalPose){};

  private:
    ugl::lie::Pose m_goalPose;

    static constexpr double m_positionTolerance = 0.1;
    static constexpr double m_headingTolerance = 0.5;
};

} // namespace pet::rrt
