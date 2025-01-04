#include "pathVerification.hpp"

#include <ugl/lie_group/pose.h>

#include <cmath>
#include <iostream>

namespace pet
{
namespace
{

bool isCloseHeading(const ugl::lie::Pose &lhs, const ugl::lie::Pose &rhs, double tolerance)
{
    const auto rotationDiff = ugl::lie::ominus(lhs.rotation(), rhs.rotation());
    const auto headingDiff  = rotationDiff[2];
    return std::abs(headingDiff) < tolerance;
}

} // namespace

void verifyVelocityContinuity(const rrt::Graph &searchTree)
{
    searchTree.forEachNode([&](const rrt::Node &node) {
        if (!rrt::isRoot(node))
        {
            // const auto &child  = node;
            // const auto &parent = searchTree.getNode(child.parentId);
            // const auto &path   = child.pathFromParent;
            // verifyVelocityContinuity(child)
        }
    });
}

void verifyHeadingContinuity(const rrt::Graph &searchTree)
{
    bool headingIsContinuous = true;
    searchTree.forEachNode([&](const rrt::Node &node) {
        if (!rrt::isRoot(node))
        {
            const auto &child  = node;
            const auto &parent = searchTree.getNode(child.parentId);
            const auto &path   = child.pathFromParent;

            static constexpr double tolerance = 1e-3;
            if (!isCloseHeading(child.state.pose, path.back().pose, tolerance))
            {
                headingIsContinuous = false;
                std::cout << "[1] Discontinuous heading found!" << std::endl;
            }
            if (!isCloseHeading(parent.state.pose, path.front().pose, tolerance))
            {
                headingIsContinuous = false;
                std::cout << "[2] Discontinuous heading found!" << std::endl;
            }
        }
    });
    if (headingIsContinuous)
    {
        std::cout << "Heading is continuous." << std::endl;
    }
}

} // namespace pet
