#include "pet_mk_vii_planner/goal.hpp"

#include <cmath>

namespace pet::rrt
{
namespace
{

template <typename ScalarType> constexpr ScalarType square(ScalarType x)
{
    return x * x;
}

} // namespace

VehicleState Goal::sampleState() const
{
    return VehicleState{m_targetPose, m_targetSpeed};
}

bool Goal::isReached(const VehicleState &state) const
{
    const auto posDiff   = m_targetPose.position() - state.pose.position();
    const auto angleDiff = m_targetPose.rotation().to_quaternion().angularDistance(
        state.pose.rotation().to_quaternion());
    const auto speedDiff = m_targetSpeed - state.velocity;
    return posDiff.squaredNorm() < square(m_positionTolerance) &&
           std::abs(angleDiff) < m_headingTolerance && std::abs(speedDiff) < m_speedTolerance;
}

} // namespace pet::rrt
