#include "velocityProfile.hpp"

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <gtest/gtest.h>

namespace pet::rrt::test
{

TEST(VelocityProfileTest, Given_NoSampleRatios_Expect_NoVelocitySamples)
{
    const auto vehicleModel    = rrt::VehicleModel{0.5, 2.0};
    const auto velocitySamples = computeVelocityProfile({}, 5.0, 0.2, 0.2, vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 0);
}

TEST(VelocityProfileTest, Given_OneSampleRatio_Expect_OneVelocitySample) {}

TEST(VelocityProfileTest, Given_TwoSampleRatios_Expect_TwoVelocitySamples) {}

TEST(VelocityProfileTest, Given_RatioEqualZero_Expect_VelocityEqualStartVelocity) {}

TEST(VelocityProfileTest, Given_RatioEqualOne_Expect_VelocityEqualEndVelocity) {}

} // namespace pet::rrt::test
