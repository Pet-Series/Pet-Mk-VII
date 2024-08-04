#include "velocityProfile.hpp"

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace pet::rrt::test
{

TEST(VelocityProfileTest, Given_NoSampleRatios_Expect_NoVelocitySamples)
{
    const auto vehicleModel    = rrt::VehicleModel{3.0, 5.0};
    const auto velocitySamples = computeVelocityProfile({}, 5.0, 2.0, 2.0, vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 0UL);
}

TEST(VelocityProfileTest, Given_OneSampleRatio_Expect_OneVelocitySample)
{
    const auto vehicleModel = rrt::VehicleModel{3.0, 5.0};
    const auto sampleRatios = std::vector{0.25};
    const auto velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, 2.0, vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 1UL);
}

TEST(VelocityProfileTest, Given_TwoSampleRatios_Expect_TwoVelocitySamples)
{
    const auto vehicleModel = rrt::VehicleModel{3.0, 5.0};
    const auto sampleRatios = std::vector{0.25, 0.75};
    const auto velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, 2.0, vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 2UL);
}

} // namespace pet::rrt::test
