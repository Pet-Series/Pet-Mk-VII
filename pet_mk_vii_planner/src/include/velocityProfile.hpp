#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <vector>

namespace pet::rrt
{

/// @brief Compute and sample from a trapezoid velocity profile.
/// @param timeRatios list of desired samples as ratios in time domain [0,1]
/// @param totalDistance total distance of profile.
/// @param startVel start velocity
/// @param endVel end velocity
/// @param vehicleModel dynamic constraints
/// @return List of velocity samples
std::vector<double> computeVelocityProfile(const std::vector<double> &timeRatios,
                                           double totalDistance, double startVel,
                                           double              endVel,
                                           const VehicleModel &vehicleModel);

} // namespace pet::rrt
