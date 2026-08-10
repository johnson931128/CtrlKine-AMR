#include "LidarSimulator.hpp"
#include "MapLikelihoodField.hpp"
#include "OdometrySimulator.hpp"
#include "LocalizationVisualization.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

#include "TestSupport.hpp"

namespace {
bool near(double first, double second, double tolerance = 0.001) {
    return std::abs(first - second) <= tolerance;
}

MapData makeMap() {
    MapData map(50.0f);
    map.setWorldBoundary(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(500.0f, 500.0f)));
    return map;
}

LidarConfig singleBeamConfig(double maximumRange = 1000.0) {
    LidarConfig config;
    config.beamCount = 1;
    config.fieldOfView = 0.0;
    config.minRange = 0.0;
    config.maxRange = maximumRange;
    config.rangeNoiseStdDev = 0.0;
    return config;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "LIDAR-001", "straight ray hits an obstacle cell surface", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{3, 2});
        std::mt19937 randomEngine(1);
        const LaserScan scan = LidarSimulator(singleBeamConfig()).simulate(
            Pose2D{sf::Vector2f(25.0f, 125.0f), 0.0f}, map, randomEngine
        );
        return scan.ranges.size() == 1 && near(scan.ranges[0], 125.0);
    });

    runTest(suite, "LIDAR-002", "nearest of multiple obstacle hits is returned", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{5, 2});
        map.addObstacle(GridCoord{3, 2});
        std::mt19937 randomEngine(2);
        const LaserScan scan = LidarSimulator(singleBeamConfig()).simulate(
            Pose2D{sf::Vector2f(25.0f, 125.0f), 0.0f}, map, randomEngine
        );
        return near(scan.ranges[0], 125.0);
    });

    runTest(suite, "LIDAR-003", "diagonal ray uses exact AABB intersection", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        std::mt19937 randomEngine(3);
        const LaserScan scan = LidarSimulator(singleBeamConfig()).simulate(
            Pose2D{sf::Vector2f(25.0f, 25.0f), static_cast<float>(kLocalizationPi / 4.0)},
            map,
            randomEngine
        );
        return near(scan.ranges[0], std::sqrt(75.0 * 75.0 * 2.0), 0.01);
    });

    runTest(suite, "LIDAR-004", "world boundary terminates a ray", [] {
        MapData map = makeMap();
        std::mt19937 randomEngine(4);
        const LaserScan scan = LidarSimulator(singleBeamConfig()).simulate(
            Pose2D{sf::Vector2f(250.0f, 250.0f), 0.0f}, map, randomEngine
        );
        return near(scan.ranges[0], 250.0);
    });

    runTest(suite, "LIDAR-005", "range is clamped at configured maximum", [] {
        MapData map = makeMap();
        std::mt19937 randomEngine(5);
        const LaserScan scan = LidarSimulator(singleBeamConfig(100.0)).simulate(
            Pose2D{sf::Vector2f(250.0f, 250.0f), 0.0f}, map, randomEngine
        );
        return near(scan.ranges[0], 100.0);
    });

    runTest(suite, "LIDAR-006", "origin on an obstacle surface returns minimum range", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        LidarConfig config = singleBeamConfig();
        config.minRange = 5.0;
        std::mt19937 randomEngine(6);
        const LaserScan scan = LidarSimulator(config).simulate(
            Pose2D{sf::Vector2f(100.0f, 125.0f), 0.0f}, map, randomEngine
        );
        return near(scan.ranges[0], 5.0);
    });

    runTest(suite, "LIDAR-007", "seeded Gaussian noise is deterministic", [] {
        MapData map = makeMap();
        LidarConfig config = singleBeamConfig();
        config.rangeNoiseStdDev = 3.0;
        LidarSimulator simulator(config);
        std::mt19937 firstEngine(77);
        std::mt19937 secondEngine(77);
        const Pose2D pose{sf::Vector2f(250.0f, 250.0f), 0.0f};
        const LaserScan first = simulator.simulate(pose, map, firstEngine);
        const LaserScan second = simulator.simulate(pose, map, secondEngine);
        return first.ranges == second.ranges && !near(first.ranges[0], 250.0, 1e-6);
    });

    runTest(suite, "LIDAR-008", "invalid configuration is rejected", [] {
        LidarConfig config;
        config.beamCount = 0;
        try {
            LidarSimulator simulator(config);
            (void)simulator;
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    });

    runTest(suite, "FIELD-001", "obstacle interior and surface have zero distance", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        return near(field.distanceAt(sf::Vector2f(125.0f, 125.0f)), 0.0)
            && near(field.distanceAt(sf::Vector2f(100.0f, 125.0f)), 0.0);
    });

    runTest(suite, "FIELD-002", "neighbor distance uses map-surface units", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        return near(field.distanceAt(sf::Vector2f(75.0f, 125.0f)), 25.0);
    });

    runTest(suite, "FIELD-003", "distance is clamped and boundary is a map surface", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        MapLikelihoodField field;
        field.rebuild(map, 60.0);
        return near(field.distanceAt(sf::Vector2f(400.0f, 250.0f)), 60.0)
            && near(field.distanceAt(sf::Vector2f(0.0f, 250.0f)), 0.0);
    });

    runTest(suite, "FIELD-004", "geometry revision rebuild removes stale distance data", [] {
        MapData map = makeMap();
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        const double before = field.distanceAt(sf::Vector2f(275.0f, 275.0f));
        const std::uint64_t previousRevision = field.getSourceRevision();
        map.addObstacle(GridCoord{5, 5});
        field.rebuildIfNeeded(map, 150.0);
        return before > 0.0
            && near(field.distanceAt(sf::Vector2f(275.0f, 275.0f)), 0.0)
            && field.getSourceRevision() > previousRevision;
    });

    runTest(suite, "FIELD-005", "clear, resolution, and boundary changes rebuild derived geometry", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        map.clear();
        field.rebuildIfNeeded(map, 150.0);
        const bool cleared = field.distanceAt(sf::Vector2f(125.0f, 125.0f)) > 0.0;
        map.setGridResolution(25.0f);
        field.rebuildIfNeeded(map, 150.0);
        const bool resolutionUpdated = field.isValid()
            && field.getSourceRevision() == map.getGeometryRevision();
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(-100.0f, -100.0f), sf::Vector2f(300.0f, 300.0f)
        ));
        field.rebuildIfNeeded(map, 150.0);
        return cleared && resolutionUpdated
            && near(field.distanceAt(sf::Vector2f(-100.0f, 0.0f)), 0.0)
            && field.getSourceRevision() == map.getGeometryRevision();
    });

    runTest(suite, "FIELD-006", "different map instances cannot reuse a matching revision by accident", [] {
        MapData first = makeMap();
        MapData second = makeMap();
        first.addObstacle(GridCoord{2, 2});
        second.addObstacle(GridCoord{6, 6});
        if (first.getGeometryRevision() != second.getGeometryRevision()) {
            return false;
        }
        MapLikelihoodField field;
        field.rebuild(first, 150.0);
        field.rebuildIfNeeded(second, 150.0);
        return near(field.distanceAt(sf::Vector2f(325.0f, 325.0f)), 0.0)
            && field.distanceAt(sf::Vector2f(125.0f, 125.0f)) > 0.0;
    });

    runTest(suite, "FIELD-007", "diagonal query matches exact obstacle-corner distance", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 2});
        MapLikelihoodField field;
        field.rebuild(map, 150.0);
        return near(
            field.distanceAt(sf::Vector2f(75.0f, 75.0f)),
            std::hypot(25.0, 25.0),
            0.001
        );
    });

    runTest(suite, "FIELD-008", "field exposes free, occupied, and outside point semantics", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(300.0f, 300.0f)
        ));
        map.addObstacle(GridCoord{2, 2});
        MapLikelihoodField field;
        field.rebuild(map, 100.0);
        return field.isFree(sf::Vector2f(25.0f, 25.0f))
            && !field.isFree(sf::Vector2f(125.0f, 125.0f))
            && !field.isFree(sf::Vector2f(-1.0f, 25.0f));
    });

    runTest(suite, "LIDAR-009", "max-range no-return remains saturated under noise", [] {
        MapData map = makeMap();
        LidarConfig config = singleBeamConfig(100.0);
        config.rangeNoiseStdDev = 20.0;
        LidarSimulator lidar(config);
        std::mt19937 randomEngine(123);
        const LaserScan scan = lidar.simulate(
            Pose2D{sf::Vector2f(250.0f, 250.0f), 0.0f}, map, randomEngine
        );
        return near(scan.ranges.front(), 100.0);
    });

    runTest(suite, "LIDAR-010", "nonzero sensor translation and yaw extrinsics affect scan geometry", [] {
        MapData map = makeMap();
        map.addObstacle(GridCoord{2, 3});
        LidarConfig config = singleBeamConfig();
        config.offsetX = 25.0;
        config.yawOffset = kLocalizationPi / 2.0;
        LidarSimulator lidar(config);
        std::mt19937 randomEngine(4);
        const LaserScan scan = lidar.simulate(
            Pose2D{sf::Vector2f(125.0f, 25.0f), 0.0f}, map, randomEngine
        );
        return near(scan.ranges.front(), 125.0)
            && near(scan.sensorOffsetX, 25.0)
            && near(scan.sensorYawOffset, kLocalizationPi / 2.0);
    });

    runTest(suite, "LIDAR-011", "extrinsic origin outside the map yields a safe invalid scan", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(500.0f, 500.0f)
        ));
        LidarConfig config;
        config.beamCount = 9;
        config.fieldOfView = kLocalizationPi;
        config.minRange = 1.0;
        config.maxRange = 200.0;
        config.offsetX = -40.0;
        config.rangeNoiseStdDev = 0.0;
        std::mt19937 engine(1234);
        const LaserScan scan = LidarSimulator(config).simulate(
            Pose2D{sf::Vector2f(20.0f, 250.0f), 0.0f}, map, engine
        );
        return scan.ranges.size() == config.beamCount
            && std::all_of(scan.ranges.begin(), scan.ranges.end(), [](float range) {
                return std::isnan(range);
            });
    });

    runTest(suite, "COV-001", "covariance ellipse uses stable eigen decomposition", [] {
        LocalizationCovariance covariance;
        covariance.values[0] = 100.0;
        covariance.values[4] = 25.0;
        const CovarianceEllipse ellipse = covarianceEllipse(covariance, 2.0);
        return ellipse.valid && near(ellipse.majorRadius, 20.0)
            && near(ellipse.minorRadius, 10.0) && near(ellipse.rotation, 0.0);
    });

    runTest(suite, "VIS-001", "default localization view is readable and inference-neutral", [] {
        const LocalizationViewOptions view;
        return view.particles && view.estimate && view.covariance
            && !view.lidarRays && !view.lidarHitPoints && !view.odometry
            && view.renderedRayCount < LidarConfig{}.beamCount;
    });

    runTest(suite, "ODOM-001", "zero accepted motion remains exactly stable", [] {
        OdometrySimulator simulator;
        const Pose2D pose{sf::Vector2f(100.0f, 100.0f), 0.4f};
        simulator.reset(pose);
        std::mt19937 randomEngine(10);
        const OdometryDelta delta = simulator.observe(pose, randomEngine);
        return delta.valid && delta.rotation1 == 0.0 && delta.translation == 0.0
            && delta.rotation2 == 0.0
            && near(simulator.getOdometryPose().position.x, 100.0)
            && near(simulator.getOdometryPose().heading, 0.4);
    });

    runTest(suite, "ODOM-002", "noise-free forward, rotation, and mixed increments decompose correctly", [] {
        OdometryConfig config{};
        config.translationStdDevPerDistance = 0.0;
        config.translationStdDevPerRotation = 0.0;
        config.rotationStdDevPerRotation = 0.0;
        config.rotationStdDevPerDistance = 0.0;
        OdometrySimulator simulator(config);
        simulator.reset(Pose2D{sf::Vector2f(0.0f, 0.0f), 0.0f});
        std::mt19937 randomEngine(11);
        const OdometryDelta forward = simulator.observe(
            Pose2D{sf::Vector2f(100.0f, 0.0f), 0.0f}, randomEngine
        );
        const OdometryDelta rotate = simulator.observe(
            Pose2D{sf::Vector2f(100.0f, 0.0f), static_cast<float>(kLocalizationPi / 2.0)},
            randomEngine
        );
        const OdometryDelta mixed = simulator.observe(
            Pose2D{sf::Vector2f(100.0f, 50.0f), static_cast<float>(kLocalizationPi / 2.0)},
            randomEngine
        );
        return near(forward.translation, 100.0) && near(forward.rotation1, 0.0)
            && near(rotate.translation, 0.0)
            && near(rotate.rotation2, kLocalizationPi / 2.0)
            && near(mixed.translation, 50.0) && near(mixed.rotation1, 0.0);
    });

    runTest(suite, "ODOM-003", "reverse motion uses signed translation without a false pi turn", [] {
        OdometryConfig config{};
        config.translationStdDevPerDistance = 0.0;
        config.translationStdDevPerRotation = 0.0;
        config.rotationStdDevPerRotation = 0.0;
        config.rotationStdDevPerDistance = 0.0;
        OdometrySimulator simulator(config);
        simulator.reset(Pose2D{sf::Vector2f(0.0f, 0.0f), 0.0f});
        std::mt19937 randomEngine(12);
        const OdometryDelta reverse = simulator.observe(
            Pose2D{sf::Vector2f(-50.0f, 0.0f), 0.0f}, randomEngine
        );
        return near(reverse.translation, -50.0)
            && near(reverse.rotation1, 0.0)
            && near(reverse.rotation2, 0.0);
    });

    runTest(suite, "ODOM-004", "seeded odometry noise is deterministic", [] {
        OdometrySimulator first;
        OdometrySimulator second;
        const Pose2D start{sf::Vector2f(0.0f, 0.0f), 0.0f};
        const Pose2D end{sf::Vector2f(100.0f, 10.0f), 0.2f};
        first.reset(start);
        second.reset(start);
        std::mt19937 firstEngine(99);
        std::mt19937 secondEngine(99);
        const OdometryDelta firstDelta = first.observe(end, firstEngine);
        const OdometryDelta secondDelta = second.observe(end, secondEngine);
        return near(firstDelta.rotation1, secondDelta.rotation1, 1e-12)
            && near(firstDelta.translation, secondDelta.translation, 1e-12)
            && near(firstDelta.rotation2, secondDelta.rotation2, 1e-12);
    });

    runTest(suite, "ODOM-005", "kidnap rebase preserves odometry belief and hides teleport delta", [] {
        OdometryConfig config{};
        config.translationStdDevPerDistance = 0.0;
        config.translationStdDevPerRotation = 0.0;
        config.rotationStdDevPerRotation = 0.0;
        config.rotationStdDevPerDistance = 0.0;
        OdometrySimulator simulator(config);
        const Pose2D poseA{sf::Vector2f(100.0f, 100.0f), 0.2f};
        const Pose2D poseB{sf::Vector2f(400.0f, 350.0f), -0.8f};
        simulator.reset(poseA);
        simulator.rebaseGroundTruthReference(poseB);
        std::mt19937 engine(55);
        const OdometryDelta delta = simulator.observe(poseB, engine);
        return near(delta.translation, 0.0) && near(delta.rotation1, 0.0)
            && near(delta.rotation2, 0.0)
            && near(simulator.getOdometryPose().position.x, poseA.position.x)
            && near(simulator.getOdometryPose().position.y, poseA.position.y)
            && near(simulator.getOdometryPose().heading, poseA.heading);
    });

    return suite.exitCode();
}
