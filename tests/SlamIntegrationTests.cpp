#include "SlamTestUtils.hpp"

#include <cmath>
#include <iostream>
#include <random>

#include "AmclLocalizer.hpp"
#include "SlamVisualization.hpp"
#include "TestSupport.hpp"

namespace {
bool sameDelta(const OdometryDelta& first, const OdometryDelta& second) {
    return first.valid == second.valid
        && first.rotation1 == second.rotation1
        && first.translation == second.translation
        && first.rotation2 == second.rotation2;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "SLAM-INTEGRATION-001", "ideal deterministic trajectory tracks in the SLAM-local frame", [] {
        OdometryConfig odometry;
        odometry.translationStdDevPerDistance = 0.0;
        odometry.translationStdDevPerRotation = 0.0;
        odometry.rotationStdDevPerRotation = 0.0;
        odometry.rotationStdDevPerDistance = 0.0;
        const SlamScenarioMetrics metrics = runSlamScenario(1, odometry, 0.0);
        std::cout << "Ideal metrics: position RMSE=" << metrics.positionRmse
                  << ", heading RMSE deg=" << metrics.headingRmseRadians * 57.2957795
                  << ", occupied IoU=" << metrics.occupiedIou
                  << ", agreement=" << metrics.occupancyAgreement
                  << ", coverage=" << metrics.knownCoverage << "\n";
        return metrics.completed && metrics.positionRmse <= 15.0
            && metrics.headingRmseRadians <= 0.08;
    });

    runTest(suite, "SLAM-INTEGRATION-002", "SLAM mapping never mutates MapData truth", [] {
        MapData map = makeSlamTestMap();
        const std::uint64_t revision = map.getGeometryRevision();
        const std::size_t obstacles = map.getObstacles().size();
        const Pose2D pose{sf::Vector2f(-120.0f, -100.0f), 0.0f};
        LidarConfig config;
        config.beamCount = 91;
        config.fieldOfView = 2.0 * kLocalizationPi;
        config.minRange = 1.0;
        config.maxRange = 200.0;
        config.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(config);
        std::mt19937 rng(4);
        const LaserScan scan = lidar.simulate(pose, map, rng);
        SlamFrontend frontend(makeSlamScenarioConfig());
        frontend.process(OdometryDelta{}, scan);
        return frontend.getMap().getRevision() == 1
            && map.getGeometryRevision() == revision
            && map.getObstacles().size() == obstacles;
    });

    runTest(suite, "SLAM-INTEGRATION-003", "deterministic SLAM consumes sensor values without mutating them", [] {
        MapData map = makeSlamTestMap();
        const Pose2D pose{sf::Vector2f(-120.0f, -100.0f), 0.0f};
        LidarConfig config;
        config.beamCount = 31;
        config.fieldOfView = 2.0 * kLocalizationPi;
        config.minRange = 1.0;
        config.maxRange = 200.0;
        config.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(config);
        std::mt19937 rng(5);
        LaserScan scan = lidar.simulate(pose, map, rng);
        const LaserScan before = scan;
        OdometryDelta delta{0.1, 12.0, -0.05, true};
        const OdometryDelta deltaBefore = delta;
        SlamFrontend frontend(makeSlamScenarioConfig());
        frontend.process(delta, scan);
        return scan.ranges == before.ranges && scan.angleMin == before.angleMin
            && scan.angleIncrement == before.angleIncrement
            && scan.sensorOffsetX == before.sensorOffsetX
            && sameDelta(delta, deltaBefore);
    });

    runTest(suite, "SLAM-SIM-001", "separate LiDAR activity cannot perturb an odometry RNG sequence", [] {
        OdometryConfig config;
        OdometrySimulator first(config);
        OdometrySimulator second(config);
        std::mt19937 firstOdometryRng(10);
        std::mt19937 secondOdometryRng(10);
        std::mt19937 extraLidarRng(11);
        const Pose2D start{sf::Vector2f(0.0f, 0.0f), 0.0f};
        first.reset(start);
        second.reset(start);
        for (int step = 1; step <= 10; ++step) {
            for (int draw = 0; draw < step * 3; ++draw) {
                (void)extraLidarRng();
            }
            const Pose2D pose{sf::Vector2f(step * 10.0f, step * 2.0f), step * 0.02f};
            if (!sameDelta(
                    first.observe(pose, firstOdometryRng),
                    second.observe(pose, secondOdometryRng))) {
                return false;
            }
        }
        return true;
    });

    runTest(suite, "SLAM-ARCH-004", "AMCL and SLAM reset state independently", [] {
        MapData map = makeSlamTestMap();
        const Pose2D pose{sf::Vector2f(-120.0f, -100.0f), 0.0f};
        std::mt19937 amclRng(12);
        AmclConfig amclConfig;
        amclConfig.initialParticleCount = 100;
        amclConfig.minParticles = 50;
        amclConfig.maxParticles = 200;
        AmclLocalizer amcl(amclConfig);
        if (!amcl.initializeLocal(pose, map, amclRng)) return false;

        LidarConfig lidarConfig;
        lidarConfig.beamCount = 31;
        lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
        lidarConfig.minRange = 1.0;
        lidarConfig.maxRange = 200.0;
        lidarConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(lidarConfig);
        std::mt19937 lidarRng(13);
        SlamFrontend slam(makeSlamScenarioConfig());
        slam.process(OdometryDelta{}, lidar.simulate(pose, map, lidarRng));
        const std::size_t particleCount = amcl.getParticles().size();
        slam.reset();
        if (amcl.getParticles().size() != particleCount) return false;
        slam.process(OdometryDelta{}, lidar.simulate(pose, map, lidarRng));
        const std::uint64_t slamRevision = slam.getMap().getRevision();
        amcl.reset();
        return slam.getMap().getRevision() == slamRevision
            && slam.getState() == SlamState::Tracking;
    });

    runTest(suite, "SLAM-SIM-002", "display transform is observational and preserves local inference state", [] {
        const Pose2D local{sf::Vector2f(20.0f, 10.0f), 0.3f};
        const Pose2D origin{sf::Vector2f(100.0f, 200.0f), 0.5f};
        const Pose2D display = SlamVisualization::toDisplayPose(local, origin);
        return std::abs(display.position.x
                - (100.0 + std::cos(0.5) * 20.0 - std::sin(0.5) * 10.0)) < 1e-4
            && std::abs(display.position.y
                - (200.0 + std::sin(0.5) * 20.0 + std::cos(0.5) * 10.0)) < 1e-4
            && std::abs(display.heading - 0.8) < 1e-6
            && local.position == sf::Vector2f(20.0f, 10.0f)
            && local.heading == 0.3f;
    });

    return suite.exitCode();
}
