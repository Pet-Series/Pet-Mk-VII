#include "pet_mk_vii_planner/goal.hpp"

namespace pet::rrt
{
namespace
{

template <typename ScalarType> constexpr ScalarType square(ScalarType x) { return x * x; }

} // namespace

ugl::lie::Pose Goal::sampleState() const { return m_goalPose; }

bool Goal::isReached(const ugl::lie::Pose &state) const
{
    const auto posDiff = m_goalPose.position() - state.position();
    const auto angleDiff = m_goalPose.rotation().to_quaternion().angularDistance(
        state.rotation().to_quaternion());
    return posDiff.squaredNorm() < square(m_positionTolerance) &&
           angleDiff < m_headingTolerance;
}

} // namespace pet::rrt
