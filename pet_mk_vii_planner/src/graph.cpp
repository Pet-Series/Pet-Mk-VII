#include "pet_mk_vii_planner/graph.hpp"

#include "utility/algorithm.hpp"

#include "utility/tiktok.hpp"

#include <ugl/lie_group/pose.h>
#include <ugl/random/uniform_distribution.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

namespace pet::rrt
{
namespace
{

template <typename ScalarType> constexpr int sign(ScalarType x) { return x < 0 ? -1 : 1; }

int getDrivingDirection(const VehicleState &start, const VehicleState &end)
{
    if (start.velocity == 0.0)
    {
        return sign(end.velocity);
    }
    else
    {
        return sign(start.velocity);
    }
}

} // namespace

Graph::Graph(const VehicleState &startingState, const BoundingBox &boundingBox)
    : m_boundingBox(boundingBox)
{
    const double widthX  = m_boundingBox.max.x() - m_boundingBox.min.x();
    const double widthY  = m_boundingBox.max.y() - m_boundingBox.min.y();
    m_numSidesX          = static_cast<int>(std::ceil(widthX / kBucketSize));
    m_numSidesY          = static_cast<int>(std::ceil(widthY / kBucketSize));
    const int numBuckets = m_numSidesX * m_numSidesY;
    m_buckets.assign(numBuckets, std::vector<Node>{});

    Node rootNode{};
    rootNode.parentId       = NodeId{-1, -1};
    rootNode.state          = startingState;
    rootNode.pathFromParent = {};
    storeNode(rootNode);
}

const Node &Graph::addNode(const VehicleState &state, const Path &pathFromParent,
                           const Node &parent)
{
    Node node{};
    node.parentId       = parent.id;
    node.state          = state;
    node.pathFromParent = pathFromParent;
    return storeNode(node);
}

Node &Graph::getNode(const NodeId &id)
{
    return m_buckets[id.bucketIndex][id.internalIndex];
}

const Node &Graph::getNode(const NodeId &id) const
{
    return m_buckets[id.bucketIndex][id.internalIndex];
}

Node Graph::findClosest(const VehicleState &targetState) const
{
    util::TikTok timer{"Graph::findClosest"};
    /// TODO: Create overload of findClosest that takes a binary distance function.
    // auto distanceFunc = [](const VehicleState &a, const VehicleState &b) -> double {
    //     return ugl::lie::ominus(a.pose, b.pose).squaredNorm();
    // };
    auto distanceFunc = [](const VehicleState &start, const VehicleState &end) -> double {
        const int          drivingDirection = getDrivingDirection(start, end);
        const double       maxCurvature     = 2.0; // TODO: Use vehicle model
        const double       minRadius        = 1.0 / maxCurvature;
        const double       tangentialOffset = 2 * minRadius;
        const ugl::Vector3 endTangent =
            ugl::Vector3::UnitX() * -drivingDirection * tangentialOffset;

        const ugl::lie::Pose desiredStartPose =
            end.pose * ugl::lie::Pose{ugl::lie::Rotation::Identity(), endTangent};

        return ugl::lie::ominus(start.pose, desiredStartPose).squaredNorm();
    };

    const double localX = targetState.pose.position().x() - m_boundingBox.min.x();
    const double localY = targetState.pose.position().y() - m_boundingBox.min.y();
    const int    indexX = static_cast<int>(std::floor(localX / kBucketSize));
    const int    indexY = static_cast<int>(std::floor(localY / kBucketSize));

    const std::array bucketOffset = {-1, 0, 1};

    Node   nearestNode;
    double minDistance = std::numeric_limits<double>::infinity();

    for (const int offsetX : bucketOffset)
    {
        const int ix = indexX + offsetX;
        if (ix < 0 || ix >= m_numSidesX)
        {
            continue;
        }
        for (const int offsetY : bucketOffset)
        {
            const int iy = indexY + offsetY;
            if (iy < 0 || iy >= m_numSidesY)
            {
                continue;
            }
            const auto &bucket = m_buckets[ix + m_numSidesX * iy];
            for (const auto &node : bucket)
            {
                const double distance = distanceFunc(node.state, targetState);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestNode = node;
                }
            }
        }
    }

    // No nodes found in neighbouring buckets. Fall back on naive serch of complete tree.
    if (minDistance == std::numeric_limits<double>::infinity())
    {
        /// TODO: Implement using Graph::forEachNode().
        for (const auto &bucket : m_buckets)
        {
            for (const auto &node : bucket)
            {
                const double distance = distanceFunc(node.state, targetState);
                if (distance < minDistance)
                {
                    minDistance = distance;
                    nearestNode = node;
                }
            }
        }
    }
    return nearestNode;
}

Node Graph::sampleClose(const VehicleState &targetState) const
{
    util::TikTok timer{"Graph::sampleClose"};

    const double localX = targetState.pose.position().x() - m_boundingBox.min.x();
    const double localY = targetState.pose.position().y() - m_boundingBox.min.y();
    const int    indexX = static_cast<int>(std::floor(localX / kBucketSize));
    const int    indexY = static_cast<int>(std::floor(localY / kBucketSize));

    const std::array bucketOffset = {-2, -1, 0, 1, 2};

    std::size_t totalPotentialNodes = 0;
    for (const int offsetX : bucketOffset)
    {
        const int ix = indexX + offsetX;
        if (ix < 0 || ix >= m_numSidesX)
        {
            continue;
        }
        for (const int offsetY : bucketOffset)
        {
            const int iy = indexY + offsetY;
            if (iy < 0 || iy >= m_numSidesY)
            {
                continue;
            }
            const auto &bucket = m_buckets[ix + m_numSidesX * iy];
            totalPotentialNodes += bucket.size();
        }
    }

    if (totalPotentialNodes != 0)
    {
        std::size_t sampledIndex =
            static_cast<int>(std::floor(ugl::random::UniformDistribution<1>::sample(
                0.0, static_cast<double>(totalPotentialNodes))));
        for (const int offsetX : bucketOffset)
        {
            const int ix = indexX + offsetX;
            if (ix < 0 || ix >= m_numSidesX)
            {
                continue;
            }
            for (const int offsetY : bucketOffset)
            {
                const int iy = indexY + offsetY;
                if (iy < 0 || iy >= m_numSidesY)
                {
                    continue;
                }

                const auto &bucket = m_buckets[ix + m_numSidesX * iy];
                if (sampledIndex < bucket.size())
                {
                    return bucket[sampledIndex];
                }
                else
                {
                    sampledIndex -= bucket.size();
                }
            }
        }
    }
    else // Fall back on sampling from full graph.
    {
        std::size_t totalPotentialNodes = 0;
        for (const auto &bucket : m_buckets)
        {
            totalPotentialNodes += bucket.size();
        }

        std::size_t sampledIndex =
            static_cast<int>(std::floor(ugl::random::UniformDistribution<1>::sample(
                0.0, static_cast<double>(totalPotentialNodes))));
        for (const auto &bucket : m_buckets)
        {
            if (sampledIndex < bucket.size())
            {
                return bucket[sampledIndex];
            }
            else
            {
                sampledIndex -= bucket.size();
            }
        }
    }
    // Unreachable
}

std::vector<Node> Graph::getPathFromRoot(const Node &node) const
{
    Node              currentNode = node;
    std::vector<Node> path{currentNode};
    while (!isRoot(currentNode))
    {
        currentNode = getNode(currentNode.parentId);
        path.push_back(currentNode);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

const Node &Graph::storeNode(const Node &node)
{
    const int bucketIndex = findBucketIndex(node);
    auto     &bucket      = m_buckets[bucketIndex];
    bucket.push_back(node);
    Node &storedNode            = bucket.back();
    storedNode.id.bucketIndex   = bucketIndex;
    storedNode.id.internalIndex = static_cast<int>(bucket.size() - 1);
    if (!isRoot(storedNode))
    {
        getNode(storedNode.parentId).childrenIds.push_back(storedNode.id);
    }
    return storedNode;
}

int Graph::findBucketIndex(const Node &node) const
{
    assert(m_boundingBox.min.x() <= node.state.pose.position().x());
    assert(m_boundingBox.max.x() >= node.state.pose.position().x());
    assert(m_boundingBox.min.y() <= node.state.pose.position().y());
    assert(m_boundingBox.max.y() >= node.state.pose.position().y());

    const double localX = node.state.pose.position().x() - m_boundingBox.min.x();
    const double localY = node.state.pose.position().y() - m_boundingBox.min.y();
    const int    indexX = static_cast<int>(std::floor(localX / kBucketSize));
    const int    indexY = static_cast<int>(std::floor(localY / kBucketSize));

    return indexX + m_numSidesX * indexY;
}

void Graph::forEachNode(const std::function<void(const Node &)> &function) const
{
    std::for_each(m_buckets.begin(), m_buckets.end(), [&function](const auto &bucket) {
        std::for_each(bucket.begin(), bucket.end(), function);
    });
}

bool isRoot(const Node &node)
{
    return (node.parentId.bucketIndex == -1) && (node.parentId.internalIndex == -1);
}

} // namespace pet::rrt
