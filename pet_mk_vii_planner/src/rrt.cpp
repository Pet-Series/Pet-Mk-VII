#include "pet_mk_vii_planner/rrt.hpp"

#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "utility/tiktok.hpp"

#include <ugl/lie_group/rotation.h>
#include <ugl/math/vector.h>
#include <ugl/random/uniform_distribution.h>

#include <Eigen/Geometry>

namespace pet::rrt
{

SearchResult search(const Goal &goal, Graph &tree, const SearchContext &context)
{
    util::TikTok      timer{"rrt::search"};
    SearchDiagnostics diag{};
    for (int i = 0; i < context.maxIterations; ++i)
    {
        ++diag.totalIterations;
        const auto sampledState = sampleState(goal, context.vehicleModel, context.searchSpace);

        // const Node &parentNode = tree.findClosest(sampledState);
        const Node &parentNode = tree.sampleClose(sampledState);

        /// TODO: Perform collision check against map.
        const auto result =
            context.steerFunction(parentNode.state, sampledState, context.vehicleModel);
        if (result.has_value())
        {
            ++diag.totalConnections;
            const auto [reachedState, pathFromParent] = result.value();
            const Node &newNode = tree.addNode(reachedState, pathFromParent, parentNode);
            if (goal.isReached(reachedState))
            {
                return SearchResult{tree.getPathFromRoot(newNode), diag};
            }
        }
    }

    return SearchResult{{}, diag};
}

VehicleState sampleState(const Goal         &goal,
                         const VehicleModel &vehicleModel,
                         const BoundingBox  &searchSpace)
{
    if (shouldSampleFromGoal())
    {
        return goal.sampleState();
    }
    else
    {
        const ugl::Vector<2> position =
            ugl::random::UniformDistribution<2>::sample(searchSpace.min, searchSpace.max);
        const double heading = ugl::random::UniformDistribution<1>::sample(0.0, 2 * M_PI);

        const ugl::UnitQuaternion quaternion{Eigen::AngleAxisd{heading, Eigen::Vector3d::UnitZ()}};

        VehicleState state{};
        state.pose.set_position({position.x(), position.y(), 0.0});
        state.pose.set_rotation(ugl::lie::Rotation{quaternion});
        state.velocity = ugl::random::UniformDistribution<1>::sample(-vehicleModel.maxSpeed,
                                                                     vehicleModel.maxSpeed);
        return state;
    }
}

bool shouldSampleFromGoal()
{
    static constexpr double goalProbability = 0.1;
    return ugl::random::UniformDistribution<1>::sample(0.0, 1.0) < goalProbability;
}

} // namespace pet::rrt
