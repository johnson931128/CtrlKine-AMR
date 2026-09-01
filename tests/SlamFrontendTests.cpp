#include "slam/SlamFrontend.hpp"

#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

#include "sensors/LidarSimulator.hpp"
#include "map/MapData.hpp"
#include "TestSupport.hpp"

namespace {
SlamFrontendConfig frontendConfig() {
    SlamFrontendConfig config;
    config.grid.resolution = 10.0;
    config.grid.originX = -500.0;
    config.grid.originY = -500.0;
    config.grid.width = 100;
    config.grid.height = 100;
    config.matcher.coarseLinearWindow = 50.0;
    config.matcher.coarseLinearStep = 10.0;
    config.matcher.coarseAngularWindow = 0.20;
    config.matcher.coarseAngularStep = 0.05;
    config.matcher.fineLinearWindow = 10.0;
    config.matcher.fineLinearStep = 5.0;
    config.matcher.fineAngularWindow = 0.02;
    config.matcher.fineAngularStep = 0.01;
    config.matcher.maximumBeams = 31;
    config.matcher.minimumUsableBeams = 5;
    config.matcher.minimumScore = 0.18;
    config.matcher.scoreSearchRadiusCells = 2;
    config.maximumOdometryTranslation = 120.0;
    config.maximumOdometryRotation = 0.60;
    config.failuresBeforeLost = 2;
    return config;
}

MapData featureMap() {
    MapData map(20.0f);
    map.setWorldBoundary(sf::FloatRect(
        sf::Vector2f(-300.0f, -300.0f), sf::Vector2f(600.0f, 600.0f)
    ));
    for (const GridCoord cell : {
            GridCoord{-9, -7}, GridCoord{-5, 5}, GridCoord{2, -8},
            GridCoord{6, 3}, GridCoord{9, -2}, GridCoord{1, 8},
            GridCoord{-11, 2}, GridCoord{4, 6}}) {
        map.addObstacle(cell);
    }
    return map;
}

LidarSimulator lidar() {
    LidarConfig config;
    config.beamCount = 61;
    config.fieldOfView = 2.0 * kLocalizationPi;
    config.minRange = 1.0;
    config.maxRange = 250.0;
    config.rangeNoiseStdDev = 0.0;
    return LidarSimulator(config);
}

LaserScan scanAt(const Pose2D& pose) {
    MapData map = featureMap();
    LidarSimulator simulator = lidar();
    std::mt19937 randomEngine(7);
    return simulator.simulate(pose, map, randomEngine);
}

OdometryDelta exactDelta(const Pose2D& from, const Pose2D& to) {
    const double dx = to.position.x - from.position.x;
    const double dy = to.position.y - from.position.y;
    const double distance = std::hypot(dx, dy);
    OdometryDelta delta;
    if (distance > 1e-9) {
        delta.rotation1 = normalizeLocalizationAngle(std::atan2(dy, dx) - from.heading);
        delta.translation = distance;
    }
    delta.rotation2 = normalizeLocalizationAngle(
        to.heading - from.heading - delta.rotation1
    );
    return delta;
}

LaserScan allMaxScan(const LaserScan& source) {
    LaserScan scan = source;
    for (float& range : scan.ranges) range = scan.maxRange;
    return scan;
}

bool near(double actual, double expected, double tolerance) {
    return std::abs(actual - expected) <= tolerance;
}
}

int main() {
    TestSuite suite;
    const Pose2D start{sf::Vector2f(-80.0f, -60.0f), 0.10f};
    const Pose2D moved{sf::Vector2f(-45.0f, -35.0f), 0.20f};
    const LaserScan firstScan = scanAt(start);
    const LaserScan movedScan = scanAt(moved);

    runTest(suite, "SLAM-FRONT-001", "invalid frontend configuration is rejected", [] {
        SlamFrontendConfig config = frontendConfig();
        config.failuresBeforeLost = 0;
        try {
            SlamFrontend frontend(config);
            (void)frontend;
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    });

    runTest(suite, "SLAM-MATCH-001", "odometry prediction preserves rotation translation rotation semantics", [] {
        const Pose2D pose{sf::Vector2f(10.0f, 20.0f), 0.2f};
        const Pose2D predicted = SlamFrontend::predictPose(
            pose, OdometryDelta{0.3, -25.0, -0.1, true}
        );
        return near(predicted.position.x, 10.0 - 25.0 * std::cos(0.5), 1e-4)
            && near(predicted.position.y, 20.0 - 25.0 * std::sin(0.5), 1e-4)
            && near(predicted.heading, 0.4, 1e-6);
    });

    runTest(suite, "SLAM-FRONT-001", "first informative scan bootstraps identity and integrates the map", [&] {
        SlamFrontend frontend(frontendConfig());
        const SlamUpdateResult result = frontend.process(OdometryDelta{}, firstScan);
        return result.state == SlamState::Tracking && result.poseValid
            && result.mapIntegrated
            && result.match.reason == ScanMatchReason::Bootstrap
            && near(result.pose.position.x, 0.0, 1e-6)
            && near(result.pose.position.y, 0.0, 1e-6)
            && near(result.pose.heading, 0.0, 1e-6)
            && frontend.getMap().getRevision() == 1;
    });

    runTest(suite, "SLAM-FRONT-001", "invalid and all-max first scans remain uninitialized", [&] {
        SlamFrontend frontend(frontendConfig());
        LaserScan invalid = firstScan;
        invalid.angleIncrement = std::numeric_limits<float>::quiet_NaN();
        const SlamUpdateResult invalidResult = frontend.process(OdometryDelta{}, invalid);
        const SlamUpdateResult maxResult = frontend.process(OdometryDelta{}, allMaxScan(firstScan));
        return invalidResult.state == SlamState::Uninitialized
            && invalidResult.match.reason == ScanMatchReason::InvalidScan
            && maxResult.state == SlamState::Uninitialized
            && maxResult.match.reason == ScanMatchReason::NoPhysicalHits
            && frontend.getMap().getRevision() == 0;
    });

    runTest(suite, "SLAM-FRONT-002", "tracking predicts, corrects, and integrates only the accepted pose", [&] {
        SlamFrontendConfig config = frontendConfig();
        config.maximumOdometryRotation = 1.0;
        SlamFrontend frontend(config);
        frontend.process(OdometryDelta{}, firstScan);
        OdometryDelta noisy = exactDelta(start, moved);
        noisy.translation += 10.0;
        const std::uint64_t before = frontend.getMap().getRevision();
        const SlamUpdateResult result = frontend.process(noisy, movedScan);
        const double trueLocalDistance = std::hypot(
            moved.position.x - start.position.x,
            moved.position.y - start.position.y
        );
        return result.state == SlamState::Tracking && result.match.accepted
            && result.mapIntegrated && frontend.getMap().getRevision() == before + 1
            && std::hypot(result.pose.position.x, result.pose.position.y)
                < trueLocalDistance + 8.0
            && std::hypot(result.predictedPose.position.x, result.predictedPose.position.y)
                > std::hypot(result.pose.position.x, result.pose.position.y);
    });

    runTest(suite, "SLAM-FRONT-002", "accepted scan is mapped at corrected rather than predicted pose", [] {
        SlamFrontendConfig config = frontendConfig();
        config.grid.resolution = 10.0;
        config.grid.originX = -100.0;
        config.grid.originY = -100.0;
        config.grid.width = 20;
        config.grid.height = 20;
        config.matcher.coarseLinearWindow = 20.0;
        config.matcher.coarseLinearStep = 10.0;
        config.matcher.coarseAngularWindow = 0.0;
        config.matcher.fineLinearWindow = 0.0;
        config.matcher.fineAngularWindow = 0.0;
        config.matcher.maximumBeams = 4;
        config.matcher.minimumUsableBeams = 2;
        config.matcher.minimumScore = 0.40;
        config.matcher.scoreSearchRadiusCells = 0;
        SlamFrontend frontend(config);
        LaserScan bootstrap;
        bootstrap.ranges = {50.0f, 50.0f, 50.0f, 50.0f};
        bootstrap.angleMin = 0.0f;
        bootstrap.angleIncrement = static_cast<float>(kLocalizationPi * 0.5);
        bootstrap.minRange = 1.0f;
        bootstrap.maxRange = 100.0f;
        frontend.process(OdometryDelta{}, bootstrap);
        LaserScan moved = bootstrap;
        moved.ranges = {40.0f, 50.0f, 60.0f, 50.0f};
        const SlamUpdateResult result = frontend.process(
            OdometryDelta{0.0, 20.0, 0.0, true}, moved
        );
        const SlamGridCoord correctedEndpoint = frontend.getMap().worldToCell(
            sf::Vector2f(50.01f, 0.0f)
        );
        const SlamGridCoord predictedEndpoint = frontend.getMap().worldToCell(
            sf::Vector2f(60.01f, 0.0f)
        );
        return result.match.accepted && near(result.pose.position.x, 10.0, 1e-4)
            && near(result.predictedPose.position.x, 20.0, 1e-4)
            && frontend.getMap().getState(correctedEndpoint) == OccupancyState::Occupied
            && frontend.getMap().getState(predictedEndpoint) != OccupancyState::Occupied;
    });

    runTest(suite, "SLAM-FRONT-002/003", "rejected scans freeze the map and enter Lost at the configured limit", [&] {
        SlamFrontend frontend(frontendConfig());
        frontend.process(OdometryDelta{}, firstScan);
        const std::uint64_t revision = frontend.getMap().getRevision();
        const LaserScan noSupport = allMaxScan(firstScan);
        const SlamUpdateResult first = frontend.process(OdometryDelta{}, noSupport);
        const SlamUpdateResult second = frontend.process(OdometryDelta{}, noSupport);
        return first.state == SlamState::Tracking && !first.mapIntegrated
            && second.state == SlamState::Lost && !second.mapIntegrated
            && second.consecutiveFailures == 2
            && frontend.getMap().getRevision() == revision;
    });

    runTest(suite, "SLAM-FRONT-003", "large odometry enters Lost immediately without map contamination", [&] {
        SlamFrontend frontend(frontendConfig());
        frontend.process(OdometryDelta{}, firstScan);
        const std::uint64_t revision = frontend.getMap().getRevision();
        const SlamUpdateResult result = frontend.process(
            OdometryDelta{0.0, 500.0, 0.0, true}, movedScan
        );
        return result.state == SlamState::Lost
            && result.match.reason == ScanMatchReason::LargeOdometry
            && !result.mapIntegrated && frontend.getMap().getRevision() == revision;
    });

    runTest(suite, "SLAM-FRONT-003", "opposing large rotations cannot hide excessive odometry motion", [&] {
        SlamFrontend frontend(frontendConfig());
        frontend.process(OdometryDelta{}, firstScan);
        const std::uint64_t revision = frontend.getMap().getRevision();
        const SlamUpdateResult result = frontend.process(
            OdometryDelta{0.5, 5.0, -0.5, true}, movedScan
        );
        return result.state == SlamState::Lost
            && result.match.reason == ScanMatchReason::LargeOdometry
            && frontend.getMap().getRevision() == revision;
    });

    runTest(suite, "SLAM-FRONT-003", "an accepted informative scan reacquires from Lost", [&] {
        SlamFrontend frontend(frontendConfig());
        frontend.process(OdometryDelta{}, firstScan);
        const LaserScan noSupport = allMaxScan(firstScan);
        frontend.process(OdometryDelta{}, noSupport);
        frontend.process(OdometryDelta{}, noSupport);
        const SlamUpdateResult recovered = frontend.process(OdometryDelta{}, firstScan);
        return recovered.state == SlamState::Tracking && recovered.match.accepted
            && recovered.mapIntegrated && recovered.consecutiveFailures == 0;
    });

    runTest(suite, "SLAM-FRONT-004", "reset clears lifecycle pose diagnostics and SLAM-owned map", [&] {
        SlamFrontend frontend(frontendConfig());
        frontend.process(OdometryDelta{}, firstScan);
        frontend.reset();
        return frontend.getState() == SlamState::Uninitialized
            && !frontend.hasPose() && frontend.getMap().getRevision() == 0
            && frontend.getLastUpdate().acceptedUpdates == 0
            && frontend.getLastUpdate().rejectedUpdates == 0;
    });

    return suite.exitCode();
}
