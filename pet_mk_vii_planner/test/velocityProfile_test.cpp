#include "velocityProfile.hpp"

#include "pet_mk_vii_planner/rrtDefinitions.hpp"

#include <gtest/gtest.h>

#include <vector>

namespace pet::rrt::test
{

class VelocityProfileTest : public testing::Test
{
  protected:
    rrt::VehicleModel m_vehicleModel{3.0, 5.0};
};

TEST_F(VelocityProfileTest, Given_NoSampleRatios_Expect_NoVelocitySamples)
{
    const auto velocitySamples =
        computeVelocityProfile({}, 5.0, 2.0, 2.0, m_vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 0UL);
}

TEST_F(VelocityProfileTest, Given_OneSampleRatio_Expect_OneVelocitySample)
{
    const auto sampleRatios = std::vector{0.25};
    const auto velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, 2.0, m_vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 1UL);
}

TEST_F(VelocityProfileTest, Given_TwoSampleRatios_Expect_TwoVelocitySamples)
{
    const auto sampleRatios = std::vector{0.25, 0.75};
    const auto velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, 2.0, m_vehicleModel);
    EXPECT_EQ(velocitySamples.size(), 2UL);
}

TEST_F(VelocityProfileTest, Given_RatioEqualZero_Expect_VelocityEqualStartVelocity)
{
    const double startVelocity = 1.5;
    const auto   sampleRatios  = std::vector{0.0, 0.5, 1.0};
    const auto   velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, startVelocity, 2.0, m_vehicleModel);
    EXPECT_EQ(velocitySamples.front(), startVelocity);
}

TEST_F(VelocityProfileTest, Given_RatioEqualOne_Expect_VelocityEqualEndVelocity)
{
    const double endVelocity  = 1.5;
    const auto   sampleRatios = std::vector{0.0, 0.5, 1.0};
    const auto   velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, endVelocity, m_vehicleModel);
    EXPECT_EQ(velocitySamples.back(), endVelocity);
}

TEST_F(VelocityProfileTest, Given_StartVelocityHigherThanMaxSpeed_Expect_EmptyOutput)
{
    m_vehicleModel.maxSpeed    = 1.0;
    const double startVelocity = 1.5;
    const auto   sampleRatios  = std::vector{0.0, 0.5, 1.0};
    const auto   velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, startVelocity, 2.0, m_vehicleModel);
    EXPECT_TRUE(velocitySamples.empty());
}

TEST_F(VelocityProfileTest, Given_EndVelocityHigherThanMaxSpeed_Expect_EmptyOutput)
{
    m_vehicleModel.maxSpeed   = 1.0;
    const double endVelocity  = 1.5;
    const auto   sampleRatios = std::vector{0.0, 0.5, 1.0};
    const auto   velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, endVelocity, m_vehicleModel);
    EXPECT_TRUE(velocitySamples.empty());
}

TEST_F(VelocityProfileTest,
       Given_StartVelocityLowerThanNegativeMaxSpeed_Expect_EmptyOutput)
{
    m_vehicleModel.maxSpeed    = 1.0;
    const double startVelocity = 1.5;
    const auto   sampleRatios  = std::vector{0.0, 0.5, 1.0};
    const auto   velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, startVelocity, 2.0, m_vehicleModel);
    EXPECT_TRUE(velocitySamples.empty());
}

TEST_F(VelocityProfileTest, Given_EndVelocityLowerThanNegativeMaxSpeed_Expect_EmptyOutput)
{
    m_vehicleModel.maxSpeed   = 1.0;
    const double endVelocity  = 1.5;
    const auto   sampleRatios = std::vector{0.0, 0.5, 1.0};
    const auto   velocitySamples =
        computeVelocityProfile(sampleRatios, 5.0, 2.0, endVelocity, m_vehicleModel);
    EXPECT_TRUE(velocitySamples.empty());
}

} // namespace pet::rrt::test
