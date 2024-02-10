#include "pet_mk_vii_planner/steerCtrv.hpp"

#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "utility/interpolation.hpp"

#include <ugl/lie_group/pose.h>
#include <ugl/math/vector.h>

#include <cmath>

namespace pet::rrt
{

std::optional<std::pair<VehicleState, Path>> steerCtrv(const VehicleState &start,
                                                       const VehicleState &desiredEnd,
                                                       const VehicleModel &vehicleModel)
{
    const ugl::Vector<6> delta = ugl::lie::ominus(desiredEnd.pose, start.pose);

    double controlDuration = 1.0;
    double forwardVel      = delta[3] / controlDuration;
    double yawrate         = delta[2] / controlDuration;

    // If speed is too low, we will not get anywhere so adding a new node is pointless.
    if (std::abs(forwardVel) < 1e-3)
    {
        return {};
    }

    const double curvature = std::abs(yawrate / forwardVel);
    if (curvature > vehicleModel.maxCurvature)
    {
        // Adjust forward velocity such that we get maximum curvature.
        const double adjustment = curvature / vehicleModel.maxCurvature;
        forwardVel *= adjustment;
        controlDuration /= adjustment;
    }

    if (std::abs(forwardVel) > vehicleModel.maxSpeed)
    {
        // Adjust forward velocity such that we get maximum speed.
        const double adjustment = vehicleModel.maxSpeed / std::abs(forwardVel);
        forwardVel *= adjustment;
        yawrate *= adjustment;
        controlDuration /= adjustment;
    }

    ugl::Vector<6> velocity = ugl::Vector<6>::Zero();
    velocity[2]             = yawrate;
    velocity[3]             = forwardVel;

    VehicleState endState;
    endState.pose     = ugl::lie::oplus(start.pose, velocity);
    endState.velocity = velocity[3];

    /// TODO: Should start time always start from zero or be based on timestamp from
    /// previous path?
    const double startTime = 0.0;
    const double endTime   = startTime + controlDuration;
    const auto   path      = util::interpolatePath(PoseStamped{start.pose, startTime},
                                                   PoseStamped{endState.pose, endTime}, 5);

    return std::pair{endState, path};
}

} // namespace pet::rrt
