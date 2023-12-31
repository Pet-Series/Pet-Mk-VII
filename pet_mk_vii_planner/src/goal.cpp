#include "pet_mk_vii_planner/goal.hpp"

namespace pet::rrt
{

ugl::lie::Pose Goal::sampleState() const { return m_goalPose; }

} // namespace pet::rrt
