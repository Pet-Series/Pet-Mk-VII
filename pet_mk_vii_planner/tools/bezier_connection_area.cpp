#include "pet_mk_vii_planner/rrtDefinitions.hpp"
#include "pet_mk_vii_planner/steerBezier.hpp"

#include <ugl/lie_group/pose.h>

#include <Eigen/Geometry>
#include <matplot/matplot.h>

#include <cmath>
#include <utility>

namespace pet
{

struct QuiverData
{
    std::vector<double> x{};
    std::vector<double> y{};
    std::vector<double> u{};
    std::vector<double> v{};
};

ugl::Vector<2> toDirectionVector(double yaw, double scale = 1.0)
{
    const auto R =
        ugl::Matrix<2, 2>{{std::cos(yaw), -std::sin(yaw)}, {std::sin(yaw), std::cos(yaw)}};
    return R * ugl::Vector<2>::UnitX() * scale;
}

bool successfulConnection(double x, double y, double yaw)
{
    const ugl::Vector3        position{x, y, 0.0};
    const ugl::UnitQuaternion orientation{Eigen::AngleAxisd{yaw, ugl::Vector3::UnitZ()}};

    const rrt::VehicleState start{ugl::lie::Pose::Identity(), 0.0, 0.0};
    const rrt::VehicleState end{ugl::lie::Pose{orientation, position}, 0.0, 0.0};
    const rrt::VehicleModel vehicleModel{};

    const auto result = steerBezierPath(start, end, vehicleModel);
    return result.has_value();
}

/// @brief Calculates successful bezier connection on a grid.
/// @return pair of quiver data [success, failure]
std::pair<QuiverData, QuiverData> calculateConnections()
{
    const double                positionRes = 0.1;
    const double                headingRes  = 2 * M_PI / 16.0;
    const pet::rrt::BoundingBox limits{{-5.0, -5.0}, {5.0, 5.0}};

    const auto xValues   = matplot::iota(limits.min.x(), positionRes, limits.max.x());
    const auto yValues   = matplot::iota(limits.min.y(), positionRes, limits.max.y());
    const auto yawValues = matplot::iota(0.0, headingRes, 2 * M_PI);

    QuiverData success{};
    QuiverData failure{};
    for (double x : xValues)
    {
        for (double y : yValues)
        {
            for (double yaw : yawValues)
            {
                static constexpr double scale = 0.2;
                const ugl::Vector<2>    dir   = toDirectionVector(yaw, scale);
                if (successfulConnection(x, y, yaw))
                {
                    success.x.push_back(x);
                    success.y.push_back(y);
                    success.u.push_back(dir.x());
                    success.v.push_back(dir.y());
                }
                else
                {
                    failure.x.push_back(x);
                    failure.y.push_back(y);
                    failure.u.push_back(dir.x());
                    failure.v.push_back(dir.y());
                }
            }
        }
    }

    return {success, failure};
}

} // namespace pet

int main()
{
    const auto [success, failure] = pet::calculateConnections();

    matplot::hold(true);
    matplot::quiver(success.x, success.y, success.u, success.v)->color("blue");

    // Plotting failure cause maplot to behave badly. Too much data?
    // matplot::quiver(failure.x, failure.y, failure.u, failure.v)->color("red");

    matplot::axis(matplot::square);

    matplot::show();
    return 0;
}
