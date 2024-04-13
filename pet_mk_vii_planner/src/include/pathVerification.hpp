#pragma once

#include "pet_mk_vii_planner/graph.hpp"

namespace pet
{

void verifyVelocityContinuity(const rrt::Graph &searchTree);
void verifyHeadingContinuity(const rrt::Graph &searchTree);

} // namespace pet
