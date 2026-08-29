#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <set>
#include <vector>

#include "LidarSimulator.hpp"
#include "OdometrySimulator.hpp"
#include "SlamFrontend.hpp"

struct SlamScenarioMetrics {
    bool completed = false;
    std::size_t frames = 0;
    std::size_t lostFrames = 0;
    std::size_t missingPoseFrames = 0;
    std::size_t acceptedMatches = 0;
    std::size_t rejectedMatches = 0;
    double positionRmse = 0.0;
    double headingRmseRadians = 0.0;
    double odometryPositionRmse = 0.0;
    double occupiedIou = 0.0;
    double occupancyAgreement = 0.0;
    double knownCoverage = 0.0;
};

inline MapData makeSlamTestMap() {
    MapData map(20.0f);
    map.setWorldBoundary(sf::FloatRect(
        sf::Vector2f(-400.0f, -400.0f), sf::Vector2f(800.0f, 800.0f)
    ));
    for (const GridCoord cell : {
            GridCoord{-3, -8}, GridCoord{-2, 0}, GridCoord{3, -2},
            GridCoord{-7, 1}, GridCoord{6, -6}, GridCoord{5, 3},
            GridCoord{0, 6}, GridCoord{9, 1}, GridCoord{-8, -6}}) {
        map.addObstacle(cell);
    }
    return map;
}

inline std::vector<Pose2D> makeSlamLocalTrajectory() {
    std::vector<Pose2D> poses;
    poses.push_back(Pose2D{});
    for (int step = 1; step <= 12; ++step) {
        poses.push_back(Pose2D{sf::Vector2f(step * 10.0f, 0.0f), 0.0f});
    }
    for (int step = 1; step <= 8; ++step) {
        poses.push_back(Pose2D{sf::Vector2f(120.0f, 0.0f), step * 0.05f});
    }
    sf::Vector2f position(120.0f, 0.0f);
    for (int step = 1; step <= 9; ++step) {
        position += sf::Vector2f(
            static_cast<float>(10.0 * std::cos(0.4)),
            static_cast<float>(10.0 * std::sin(0.4))
        );
        poses.push_back(Pose2D{position, 0.4f});
    }
    for (int step = 1; step <= 8; ++step) {
        poses.push_back(Pose2D{position, 0.4f + step * 0.10f});
    }
    const float finalHeading = 1.2f;
    for (int step = 1; step <= 8; ++step) {
        position += sf::Vector2f(
            static_cast<float>(10.0 * std::cos(finalHeading)),
            static_cast<float>(10.0 * std::sin(finalHeading))
        );
        poses.push_back(Pose2D{position, finalHeading});
    }
    return poses;
}

inline MapData makeSlamCorridorMap() {
    MapData map(20.0f);
    map.setWorldBoundary(sf::FloatRect(
        sf::Vector2f(-400.0f, -400.0f), sf::Vector2f(800.0f, 800.0f)
    ));
    for (const GridCoord cell : {
            GridCoord{-4, -9}, GridCoord{0, -8}, GridCoord{4, -6},
            GridCoord{7, -3}, GridCoord{8, 1}, GridCoord{5, 4},
            GridCoord{1, 5}, GridCoord{-5, 3}, GridCoord{-8, -1}}) {
        map.addObstacle(cell);
    }
    return map;
}

inline std::vector<Pose2D> makeSlamCorridorTrajectory() {
    std::vector<Pose2D> poses;
    poses.push_back(Pose2D{});
    for (int step = 1; step <= 14; ++step) {
        poses.push_back(Pose2D{sf::Vector2f(step * 10.0f, 0.0f), 0.0f});
    }
    for (int step = 1; step <= 10; ++step) {
        poses.push_back(Pose2D{sf::Vector2f(140.0f, 0.0f), step * 0.08f});
    }
    sf::Vector2f position(140.0f, 0.0f);
    for (int step = 1; step <= 12; ++step) {
        position += sf::Vector2f(
            static_cast<float>(10.0 * std::cos(0.8)),
            static_cast<float>(10.0 * std::sin(0.8))
        );
        poses.push_back(Pose2D{position, 0.8f});
    }
    return poses;
}

inline Pose2D localToWorldPose(const Pose2D& local, const Pose2D& origin) {
    const double cosine = std::cos(origin.heading);
    const double sine = std::sin(origin.heading);
    return Pose2D{
        sf::Vector2f(
            origin.position.x + static_cast<float>(cosine * local.position.x - sine * local.position.y),
            origin.position.y + static_cast<float>(sine * local.position.x + cosine * local.position.y)
        ),
        static_cast<float>(normalizeLocalizationAngle(origin.heading + local.heading))
    };
}

inline Pose2D worldToLocalPose(const Pose2D& world, const Pose2D& origin) {
    const double cosine = std::cos(origin.heading);
    const double sine = std::sin(origin.heading);
    const double dx = world.position.x - origin.position.x;
    const double dy = world.position.y - origin.position.y;
    return Pose2D{
        sf::Vector2f(
            static_cast<float>(cosine * dx + sine * dy),
            static_cast<float>(-sine * dx + cosine * dy)
        ),
        static_cast<float>(normalizeLocalizationAngle(world.heading - origin.heading))
    };
}

inline SlamFrontendConfig makeSlamScenarioConfig() {
    SlamFrontendConfig config;
    config.grid.resolution = 10.0;
    config.grid.originX = -400.0;
    config.grid.originY = -400.0;
    config.grid.width = 100;
    config.grid.height = 100;
    config.matcher.coarseLinearWindow = 40.0;
    config.matcher.coarseLinearStep = 10.0;
    config.matcher.coarseAngularWindow = 0.16;
    config.matcher.coarseAngularStep = 0.04;
    config.matcher.fineLinearWindow = 10.0;
    config.matcher.fineLinearStep = 5.0;
    config.matcher.fineAngularWindow = 0.02;
    config.matcher.fineAngularStep = 0.01;
    config.matcher.maximumBeams = 45;
    config.matcher.minimumUsableBeams = 5;
    config.matcher.minimumScore = 0.16;
    config.matcher.scoreSearchRadiusCells = 2;
    config.maximumOdometryTranslation = 100.0;
    config.maximumOdometryRotation = 0.60;
    config.failuresBeforeLost = 3;
    return config;
}

inline SlamScenarioMetrics evaluateSlamMap(
    const SlamFrontend& frontend,
    const MapData& truthMap,
    const Pose2D& worldOrigin,
    SlamScenarioMetrics metrics
) {
    const SlamOccupancyGrid& estimate = frontend.getMap();
    const auto& config = estimate.getConfig();
    std::set<std::size_t> expectedOccupied;
    for (const GridCoord& obstacle : truthMap.getObstacles()) {
        const sf::Vector2f topLeft = truthMap.getMapper().gridToWorldTopLeft(obstacle);
        for (double y = topLeft.y + config.resolution * 0.5;
             y < topLeft.y + truthMap.getGridResolution(); y += config.resolution) {
            for (double x = topLeft.x + config.resolution * 0.5;
                 x < topLeft.x + truthMap.getGridResolution(); x += config.resolution) {
                const Pose2D local = worldToLocalPose(
                    Pose2D{sf::Vector2f(static_cast<float>(x), static_cast<float>(y)), 0.0f},
                    worldOrigin
                );
                if (estimate.contains(local.position)) {
                    const SlamGridCoord cell = estimate.worldToCell(local.position);
                    expectedOccupied.insert(static_cast<std::size_t>(cell.row) * config.width
                        + static_cast<std::size_t>(cell.col));
                }
            }
        }
    }

    std::size_t truePositive = 0;
    std::size_t falsePositive = 0;
    std::size_t known = 0;
    std::size_t correctKnown = 0;
    std::size_t eligible = 0;
    std::set<std::size_t> predictedOccupied;
    for (std::size_t row = 0; row < config.height; ++row) {
        for (std::size_t col = 0; col < config.width; ++col) {
            const SlamGridCoord cell{static_cast<int>(col), static_cast<int>(row)};
            const Pose2D world = localToWorldPose(
                Pose2D{estimate.cellCenter(cell), 0.0f}, worldOrigin
            );
            if (!truthMap.containsWorldPoint(world.position)) {
                continue;
            }
            ++eligible;
            const OccupancyState state = estimate.getState(cell);
            if (state == OccupancyState::Unknown) {
                continue;
            }
            ++known;
            const bool expected = truthMap.isObstacleAt(world.position);
            const bool occupied = state == OccupancyState::Occupied;
            if (occupied) {
                const std::size_t index = row * config.width + col;
                predictedOccupied.insert(index);
                if (expected) ++truePositive; else ++falsePositive;
            }
            if (occupied == expected) {
                ++correctKnown;
            }
        }
    }
    std::size_t falseNegative = 0;
    for (const std::size_t expected : expectedOccupied) {
        if (predictedOccupied.count(expected) == 0) {
            ++falseNegative;
        }
    }
    const std::size_t unionCount = truePositive + falsePositive + falseNegative;
    metrics.occupiedIou = unionCount == 0 ? 0.0
        : static_cast<double>(truePositive) / unionCount;
    metrics.occupancyAgreement = known == 0 ? 0.0
        : static_cast<double>(correctKnown) / known;
    metrics.knownCoverage = eligible == 0 ? 0.0
        : static_cast<double>(known) / eligible;
    return metrics;
}

inline SlamScenarioMetrics runSlamScenarioDefinition(
    std::uint32_t seed,
    const OdometryConfig& odometryConfig,
    double lidarNoiseStdDev,
    const MapData& truthMap,
    const Pose2D& worldOrigin,
    const std::vector<Pose2D>& localTrajectory
) {
    LidarConfig lidarConfig;
    lidarConfig.beamCount = 91;
    lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
    lidarConfig.minRange = 1.0;
    lidarConfig.maxRange = 200.0;
    lidarConfig.rangeNoiseStdDev = lidarNoiseStdDev;
    LidarSimulator lidar(lidarConfig);
    OdometrySimulator odometry(odometryConfig);
    SlamFrontend frontend(makeSlamScenarioConfig());
    std::mt19937 odometryRng(seed ^ 0x13579BDFu);
    std::mt19937 lidarRng(seed ^ 0x2468ACE0u);
    odometry.reset(localToWorldPose(localTrajectory.front(), worldOrigin));

    double positionSquaredError = 0.0;
    double headingSquaredError = 0.0;
    double odometryPositionSquaredError = 0.0;
    std::size_t errorSamples = 0;
    Pose2D odometryLocalPose;
    SlamScenarioMetrics metrics;
    metrics.frames = localTrajectory.size();
    for (const Pose2D& localTruth : localTrajectory) {
        const Pose2D worldTruth = localToWorldPose(localTruth, worldOrigin);
        const OdometryDelta delta = odometry.observe(worldTruth, odometryRng);
        odometryLocalPose = SlamFrontend::predictPose(odometryLocalPose, delta);
        const LaserScan scan = lidar.simulate(worldTruth, truthMap, lidarRng);
        const SlamUpdateResult update = frontend.process(delta, scan);
        if (update.state == SlamState::Lost) {
            ++metrics.lostFrames;
        }
        if (!update.poseValid) {
            ++metrics.missingPoseFrames;
            continue;
        }
        if (update.match.reason == ScanMatchReason::Accepted) {
            ++metrics.acceptedMatches;
        } else if (update.match.reason != ScanMatchReason::Bootstrap) {
            ++metrics.rejectedMatches;
        }
        const double dx = update.pose.position.x - localTruth.position.x;
        const double dy = update.pose.position.y - localTruth.position.y;
        const double headingError = normalizeLocalizationAngle(
            update.pose.heading - localTruth.heading
        );
        positionSquaredError += dx * dx + dy * dy;
        headingSquaredError += headingError * headingError;
        const double odometryDx = odometryLocalPose.position.x - localTruth.position.x;
        const double odometryDy = odometryLocalPose.position.y - localTruth.position.y;
        odometryPositionSquaredError += odometryDx * odometryDx + odometryDy * odometryDy;
        ++errorSamples;
    }
    if (errorSamples > 0) {
        metrics.positionRmse = std::sqrt(positionSquaredError / errorSamples);
        metrics.headingRmseRadians = std::sqrt(headingSquaredError / errorSamples);
        metrics.odometryPositionRmse = std::sqrt(
            odometryPositionSquaredError / errorSamples
        );
    }
    metrics.completed = frontend.getState() == SlamState::Tracking
        && metrics.lostFrames == 0 && metrics.missingPoseFrames == 0
        && errorSamples == localTrajectory.size();
    return evaluateSlamMap(frontend, truthMap, worldOrigin, metrics);
}

inline SlamScenarioMetrics runSlamScenario(
    std::uint32_t seed,
    const OdometryConfig& odometryConfig,
    double lidarNoiseStdDev
) {
    return runSlamScenarioDefinition(
        seed,
        odometryConfig,
        lidarNoiseStdDev,
        makeSlamTestMap(),
        Pose2D{sf::Vector2f(-120.0f, -100.0f), 0.0f},
        makeSlamLocalTrajectory()
    );
}

inline SlamScenarioMetrics runRotatedCorridorScenario(
    std::uint32_t seed,
    const OdometryConfig& odometryConfig,
    double lidarNoiseStdDev
) {
    return runSlamScenarioDefinition(
        seed,
        odometryConfig,
        lidarNoiseStdDev,
        makeSlamCorridorMap(),
        Pose2D{sf::Vector2f(-160.0f, -140.0f), 0.35f},
        makeSlamCorridorTrajectory()
    );
}
