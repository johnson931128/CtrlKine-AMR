#include "LidarSimulator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace {
constexpr double kParallelEpsilon = 1e-12;

bool finiteRect(const sf::FloatRect& rect) {
    return std::isfinite(rect.position.x) && std::isfinite(rect.position.y)
        && std::isfinite(rect.size.x) && std::isfinite(rect.size.y)
        && rect.size.x > 0.0f && rect.size.y > 0.0f;
}

bool rayRectangleInterval(
    double originX,
    double originY,
    double directionX,
    double directionY,
    const sf::FloatRect& rectangle,
    double& entry,
    double& exit
) {
    double minimum = -std::numeric_limits<double>::infinity();
    double maximum = std::numeric_limits<double>::infinity();

    const double rectangleMinimum[2] = {
        rectangle.position.x,
        rectangle.position.y
    };
    const double rectangleMaximum[2] = {
        rectangle.position.x + rectangle.size.x,
        rectangle.position.y + rectangle.size.y
    };
    const double origin[2] = {originX, originY};
    const double direction[2] = {directionX, directionY};

    for (int axis = 0; axis < 2; ++axis) {
        if (std::abs(direction[axis]) <= kParallelEpsilon) {
            if (origin[axis] < rectangleMinimum[axis] || origin[axis] > rectangleMaximum[axis]) {
                return false;
            }
            continue;
        }

        double first = (rectangleMinimum[axis] - origin[axis]) / direction[axis];
        double second = (rectangleMaximum[axis] - origin[axis]) / direction[axis];
        if (first > second) {
            std::swap(first, second);
        }
        minimum = std::max(minimum, first);
        maximum = std::min(maximum, second);
        if (minimum > maximum) {
            return false;
        }
    }

    entry = minimum;
    exit = maximum;
    return maximum >= 0.0;
}

void validateConfig(const LidarConfig& config) {
    const bool valid = config.beamCount > 0
        && std::isfinite(config.fieldOfView)
        && config.fieldOfView >= 0.0
        && config.fieldOfView <= 2.0 * kLocalizationPi
        && std::isfinite(config.minRange)
        && std::isfinite(config.maxRange)
        && config.minRange >= 0.0
        && config.minRange < config.maxRange
        && std::isfinite(config.offsetX)
        && std::isfinite(config.offsetY)
        && std::isfinite(config.yawOffset)
        && std::isfinite(config.rangeNoiseStdDev)
        && config.rangeNoiseStdDev >= 0.0;
    if (!valid) {
        throw std::invalid_argument("Lidar configuration is invalid.");
    }
}
}

LidarSimulator::LidarSimulator(const LidarConfig& config)
    : m_config(config) {
    validateConfig(m_config);
}

const LidarConfig& LidarSimulator::getConfig() const {
    return m_config;
}

LaserScan LidarSimulator::simulate(
    const Pose2D& groundTruthPose,
    const MapData& mapData,
    std::mt19937& randomEngine
) const {
    if (!std::isfinite(groundTruthPose.position.x)
        || !std::isfinite(groundTruthPose.position.y)
        || !std::isfinite(groundTruthPose.heading)
        || !finiteRect(mapData.getWorldBoundary())
        || !mapData.containsWorldPoint(groundTruthPose.position)) {
        throw std::invalid_argument("LiDAR pose or map boundary is invalid.");
    }

    LaserScan scan;
    scan.ranges.resize(m_config.beamCount);
    scan.angleMin = static_cast<float>(
        m_config.beamCount == 1
            ? 0.0
            : -(m_config.fieldOfView * 0.5)
    );
    scan.angleIncrement = static_cast<float>(
        m_config.beamCount == 1
            ? 0.0
            : m_config.fieldOfView / static_cast<double>(m_config.beamCount - 1)
    );
    scan.minRange = static_cast<float>(m_config.minRange);
    scan.maxRange = static_cast<float>(m_config.maxRange);
    scan.sensorOffsetX = m_config.offsetX;
    scan.sensorOffsetY = m_config.offsetY;
    scan.sensorYawOffset = m_config.yawOffset;

    const double cosine = std::cos(groundTruthPose.heading);
    const double sine = std::sin(groundTruthPose.heading);
    const sf::Vector2f sensorOrigin(
        groundTruthPose.position.x + static_cast<float>(cosine * m_config.offsetX - sine * m_config.offsetY),
        groundTruthPose.position.y + static_cast<float>(sine * m_config.offsetX + cosine * m_config.offsetY)
    );
    if (!mapData.containsWorldPoint(sensorOrigin)) {
        std::fill(
            scan.ranges.begin(), scan.ranges.end(),
            std::numeric_limits<float>::quiet_NaN()
        );
        return scan;
    }

    std::normal_distribution<double> rangeNoise(
        0.0,
        m_config.rangeNoiseStdDev > 0.0 ? m_config.rangeNoiseStdDev : 1.0
    );
    const double gridSize = mapData.getGridResolution();

    for (std::size_t beam = 0; beam < m_config.beamCount; ++beam) {
        const double relativeAngle = scan.angleMin + scan.angleIncrement * static_cast<double>(beam);
        const double worldAngle = groundTruthPose.heading + m_config.yawOffset + relativeAngle;
        const double directionX = std::cos(worldAngle);
        const double directionY = std::sin(worldAngle);

        double boundaryEntry = 0.0;
        double boundaryExit = 0.0;
        if (!rayRectangleInterval(
                sensorOrigin.x,
                sensorOrigin.y,
                directionX,
                directionY,
                mapData.getWorldBoundary(),
                boundaryEntry,
                boundaryExit)) {
            throw std::runtime_error("LiDAR ray did not intersect the world boundary.");
        }

        double nearestRange = std::min(m_config.maxRange, std::max(0.0, boundaryExit));
        for (const GridCoord& obstacle : mapData.getObstacles()) {
            const sf::Vector2f topLeft = mapData.getMapper().gridToWorldTopLeft(obstacle);
            const sf::FloatRect obstacleBounds(
                topLeft,
                sf::Vector2f(static_cast<float>(gridSize), static_cast<float>(gridSize))
            );
            double obstacleEntry = 0.0;
            double obstacleExit = 0.0;
            if (rayRectangleInterval(
                    sensorOrigin.x,
                    sensorOrigin.y,
                    directionX,
                    directionY,
                    obstacleBounds,
                    obstacleEntry,
                    obstacleExit)) {
                nearestRange = std::min(nearestRange, std::max(0.0, obstacleEntry));
            }
        }

        const bool maximumRangeReturn = nearestRange >= m_config.maxRange - 1e-9;
        if (m_config.rangeNoiseStdDev > 0.0 && !maximumRangeReturn) {
            nearestRange += rangeNoise(randomEngine);
        }
        scan.ranges[beam] = static_cast<float>(
            std::clamp(nearestRange, m_config.minRange, m_config.maxRange)
        );
    }

    return scan;
}
