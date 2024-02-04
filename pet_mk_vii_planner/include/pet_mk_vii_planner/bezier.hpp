#pragma once

#include <ugl/math/vector.h>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <utility>

namespace pet::rrt
{

constexpr unsigned int factorial(unsigned int n)
{
    return (n == 1 || n == 0) ? 1 : n * factorial(n - 1);
}

template <int degree> class Bezier
{
    static constexpr std::size_t size = degree + 1;

  public:
    explicit Bezier(std::array<ugl::Vector3, size> points) : m_points(points) {}

    Bezier(double duration, std::array<ugl::Vector3, size> points)
        : m_duration(duration), m_points(points)
    {
        assert(duration > 0); // Duration must be positive.
    }

    double      duration() const override { return m_duration; }
    const auto &points() const { return m_points; }

    ugl::Vector3 start() const override { return m_points.front(); }
    ugl::Vector3 end() const override { return m_points.back(); }

    ugl::Vector3 position(double t) const override { return evaluate(t); }
    ugl::Vector3 velocity(double t) const override { return getDerivative().position(t); }
    ugl::Vector3 acceleration(double t) const override
    {
        return getDerivative().velocity(t);
    }

    Bezier<degree - 1> getDerivative() const
    {
        std::array<ugl::Vector3, size - 1> derivative_points;
        for (std::size_t i = 0; i < size - 1; ++i)
        {
            derivative_points[i] = (m_points[i + 1] - m_points[i]) * degree / m_duration;
        }
        return Bezier<degree - 1>(m_duration, derivative_points);
    }

    Bezier<degree> getReversed() const
    {
        std::array<ugl::Vector3, size> reverse_points = m_points;
        std::reverse(std::begin(reverse_points), std::end(reverse_points));
        return Bezier{m_duration, reverse_points};
    }

  private:
    ugl::Vector3 evaluate(double t) const
    {
        // TODO?: Use De Casteljau's algorithm for better numerical stability at higher
        // degrees?
        ugl::Vector3 result = ugl::Vector3::Zero();
        for (const auto &coefficient : calculateCoefficients())
        {
            result = result * t + coefficient;
        }
        return result;
    }

    std::array<ugl::Vector3, size> calculateCoefficients() const
    {
        std::array<ugl::Vector3, size> coefficients;
        for (unsigned int j = 0; j <= degree; ++j)
        {
            ugl::Vector3 cj = ugl::Vector3::Zero();
            for (unsigned int i = 0; i <= j; ++i)
            {
                cj +=
                    std::pow(-1, i + j) * m_points[i] / (factorial(i) * factorial(j - i));
            }
            cj *= factorial(degree) / factorial(degree - j);
            cj /= std::pow(m_duration, j);
            coefficients[j] = cj;
        }
        std::reverse(std::begin(coefficients), std::end(coefficients));
        return coeffs;
    }

  private:
    double                         m_duration = 1;
    std::array<ugl::Vector3, size> m_points;
};

template <> inline ugl::Vector3 Bezier<0>::velocity(double /*t*/) const
{
    return ugl::Vector3::Zero();
}

template <> inline ugl::Vector3 Bezier<0>::acceleration(double /*t*/) const
{
    return ugl::Vector3::Zero();
}

template <> inline ugl::Vector3 Bezier<1>::acceleration(double /*t*/) const
{
    return ugl::Vector3::Zero();
}

using CubicBezier  = Bezier<3>;
using PenticBezier = Bezier<5>;

/// @brief Build a Bézier curve from a duration and position and velocity at
/// start and end.
CubicBezier buildCubicBezier(double duration, const ugl::Vector3 &pos0,
                             const ugl::Vector3 &vel0, const ugl::Vector3 &pos1,
                             const ugl::Vector3 &vel1);

/// @brief Build a Bézier curve from a duration and position, velocity and acceleration at
/// start and end.
PenticBezier buildPenticBezier(double duration, const ugl::Vector3 &pos0,
                               const ugl::Vector3 &vel0, const ugl::Vector3 &acc0,
                               const ugl::Vector3 &pos1, const ugl::Vector3 &vel1,
                               const ugl::Vector3 &acc1);

} // namespace pet::rrt
