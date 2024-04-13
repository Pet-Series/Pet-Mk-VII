#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <optional>
#include <utility>

namespace pet::rrt
{

std::optional<std::pair<VehicleState, Path>>
steerBezierPath(const VehicleState &start, const VehicleState &desiredEnd,
                const VehicleModel &vehicleModel);

} // namespace pet::rrt
