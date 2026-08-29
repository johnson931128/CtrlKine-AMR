#include "CorrelativeScanMatcher.hpp"

#include <cmath>
#include <stdexcept>

#include "OccupancyGridMapper.hpp"
#include "TestSupport.hpp"

namespace {
SlamOccupancyGridConfig gridConfig() {
    SlamOccupancyGridConfig config;
    config.resolution = 10.0;
    config.originX = -300.0;
    config.originY = -300.0;
    config.width = 60;
    config.height = 60;
    config.occupiedLogOddsIncrement = 1.0;
    config.freeLogOddsIncrement = -0.4;
    config.freeLogOddsThreshold = -0.2;
    config.occupiedLogOddsThreshold = 0.2;
    return config;
}

CorrelativeScanMatcherConfig matcherConfig() {
    CorrelativeScanMatcherConfig config;
    config.coarseLinearWindow = 30.0;
    config.coarseLinearStep = 10.0;
    config.coarseAngularWindow = 0.15;
    config.coarseAngularStep = 0.05;
    config.fineLinearWindow = 0.0;
    config.fineLinearStep = 5.0;
    config.fineAngularWindow = 0.0;
    config.fineAngularStep = 0.01;
    config.maximumBeams = 7;
    config.minimumUsableBeams = 4;
    config.minimumScore = 0.80;
    config.scoreSearchRadiusCells = 0;
    return config;
}

LaserScan asymmetricScan() {
    LaserScan scan;
    scan.ranges = {70.0f, 90.0f, 55.0f, 100.0f, 65.0f, 80.0f, 60.0f};
    scan.angleMin = -1.2f;
    scan.angleIncrement = 0.4f;
    scan.minRange = 1.0f;
    scan.maxRange = 150.0f;
    return scan;
}

SlamOccupancyGrid mappedGrid(const LaserScan& scan, const Pose2D& pose = Pose2D{}) {
    SlamOccupancyGrid grid(gridConfig());
    OccupancyGridMapper mapper;
    mapper.integrate(grid, scan, pose);
    return grid;
}

bool near(double actual, double expected, double tolerance = 1e-5) {
    return std::abs(actual - expected) <= tolerance;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "SLAM-MATCH-001", "invalid matcher configuration is rejected", [] {
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearStep = 0.0;
        try {
            CorrelativeScanMatcher matcher(config);
            (void)matcher;
            return false;
        } catch (const std::invalid_argument&) {
            return true;
        }
    });

    runTest(suite, "SLAM-MATCH-002/003", "correct pose scores above a displaced prediction", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearWindow = 0.0;
        config.coarseAngularWindow = 0.0;
        config.minimumScore = 0.0;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult correct = matcher.match(grid, scan, Pose2D{});
        const ScanMatchResult displaced = matcher.match(
            grid, scan, Pose2D{sf::Vector2f(30.0f, 20.0f), 0.15f}
        );
        return correct.accepted && correct.score > displaced.score;
    });

    runTest(suite, "SLAM-MATCH-003", "manual occupancy fixture validates endpoint geometry independently", [] {
        SlamOccupancyGrid grid(gridConfig());
        grid.applyEvidence({
            {{35, 30}, true},
            {{30, 36}, true},
            {{22, 30}, true}
        });
        LaserScan scan;
        scan.ranges = {50.0f, 60.0f, 70.0f};
        scan.angleMin = 0.0f;
        scan.angleIncrement = static_cast<float>(kLocalizationPi * 0.5);
        scan.minRange = 1.0f;
        scan.maxRange = 100.0f;
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearWindow = 0.0;
        config.coarseAngularWindow = 0.0;
        config.maximumBeams = 3;
        config.minimumUsableBeams = 3;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(grid, scan, Pose2D{});
        return result.accepted && result.usedBeams == 3 && result.score == 1.0;
    });

    runTest(suite, "SLAM-MATCH-002/004", "bounded search recovers combined x y and yaw error", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcher matcher(matcherConfig());
        const ScanMatchResult result = matcher.match(
            grid, scan, Pose2D{sf::Vector2f(20.0f, -10.0f), 0.10f}
        );
        return result.accepted && near(result.correctionX, -20.0)
            && near(result.correctionY, 10.0)
            && near(result.correctionYaw, -0.10)
            && near(result.correctedPose.position.x, 0.0)
            && near(result.correctedPose.position.y, 0.0)
            && near(result.correctedPose.heading, 0.0)
            && result.score >= 0.99;
    });

    runTest(suite, "SLAM-MATCH-002", "candidate counts are deterministic and bounded", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcher matcher(matcherConfig());
        const ScanMatchResult first = matcher.match(grid, scan, Pose2D{});
        const ScanMatchResult second = matcher.match(grid, scan, Pose2D{});
        return first.coarseCandidates == 343 && first.fineCandidates == 1
            && second.coarseCandidates == first.coarseCandidates
            && second.fineCandidates == first.fineCandidates
            && near(first.score, second.score)
            && near(first.correctionX, second.correctionX)
            && near(first.correctionYaw, second.correctionYaw);
    });

    runTest(suite, "SLAM-MATCH-002", "fine stage refines between coarse translation samples", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearWindow = 20.0;
        config.coarseLinearStep = 20.0;
        config.coarseAngularWindow = 0.0;
        config.fineLinearWindow = 10.0;
        config.fineLinearStep = 5.0;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(
            grid, scan, Pose2D{sf::Vector2f(5.0f, 0.0f), 0.0f}
        );
        return result.accepted && near(result.correctionX, -5.0)
            && result.fineCandidates > 1;
    });

    runTest(suite, "SLAM-MATCH-002", "deterministic ties prefer the smallest correction", [] {
        LaserScan scan;
        scan.ranges = {50.0f};
        scan.angleMin = 0.0f;
        scan.angleIncrement = 0.0f;
        scan.minRange = 1.0f;
        scan.maxRange = 100.0f;
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearWindow = 5.0;
        config.coarseLinearStep = 5.0;
        config.coarseAngularWindow = 0.0;
        config.maximumBeams = 1;
        config.minimumUsableBeams = 1;
        config.minimumScore = 0.0;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(grid, scan, Pose2D{});
        return result.accepted && near(result.correctionX, 0.0)
            && near(result.correctionY, 0.0) && near(result.correctionYaw, 0.0);
    });

    runTest(suite, "SLAM-MATCH-002", "normalized ties do not prefer yaw because of grid resolution", [] {
        SlamOccupancyGrid grid(gridConfig());
        grid.applyEvidence({{{39, 30}, true}, {{39, 29}, true}});
        LaserScan scan;
        scan.ranges = {100.0f};
        scan.angleMin = 0.0f;
        scan.angleIncrement = 0.0f;
        scan.minRange = 1.0f;
        scan.maxRange = 150.0f;
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearWindow = 10.0;
        config.coarseLinearStep = 10.0;
        config.coarseAngularWindow = 0.10;
        config.coarseAngularStep = 0.10;
        config.fineLinearWindow = 0.0;
        config.fineAngularWindow = 0.0;
        config.maximumBeams = 1;
        config.minimumUsableBeams = 1;
        config.minimumScore = 0.5;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(grid, scan, Pose2D{});
        return result.accepted && near(result.correctionX, -10.0)
            && near(result.correctionYaw, 0.0);
    });

    runTest(suite, "SLAM-MATCH-003/004", "invalid and all-max scans are rejected explicitly", [] {
        SlamOccupancyGrid grid(gridConfig());
        CorrelativeScanMatcher matcher(matcherConfig());
        LaserScan invalid = asymmetricScan();
        invalid.maxRange = invalid.minRange;
        LaserScan allMax = asymmetricScan();
        for (float& range : allMax.ranges) range = allMax.maxRange;
        const ScanMatchResult invalidResult = matcher.match(grid, invalid, Pose2D{});
        const ScanMatchResult maxResult = matcher.match(grid, allMax, Pose2D{});
        return !invalidResult.attempted
            && invalidResult.reason == ScanMatchReason::InvalidScan
            && !maxResult.attempted
            && maxResult.reason == ScanMatchReason::NoPhysicalHits;
    });

    runTest(suite, "SLAM-MATCH-003/004", "outside prediction is safe and reports insufficient support", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcher matcher(matcherConfig());
        const ScanMatchResult result = matcher.match(
            grid, scan, Pose2D{sf::Vector2f(1000.0f, 1000.0f), 0.0f}
        );
        return result.attempted && !result.accepted && result.usedBeams == 0
            && result.reason == ScanMatchReason::InsufficientMapSupport;
    });

    runTest(suite, "SLAM-MATCH-004", "unknown map cannot satisfy geometric support even with zero score threshold", [] {
        SlamOccupancyGrid grid(gridConfig());
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.minimumScore = 0.0;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(grid, asymmetricScan(), Pose2D{});
        return result.attempted && !result.accepted && result.usedBeams == 0
            && result.reason == ScanMatchReason::InsufficientMapSupport;
    });

    runTest(suite, "SLAM-MATCH-004", "mapped but displaced endpoints report PoorScore", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.coarseLinearWindow = 0.0;
        config.coarseAngularWindow = 0.0;
        config.minimumScore = 0.90;
        config.scoreSearchRadiusCells = 2;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(
            grid, scan, Pose2D{sf::Vector2f(20.0f, 0.0f), 0.0f}
        );
        return result.attempted && !result.accepted
            && result.usedBeams >= config.minimumUsableBeams
            && result.score < config.minimumScore
            && result.reason == ScanMatchReason::PoorScore;
    });

    runTest(suite, "SLAM-MATCH-003", "matching never mutates the occupancy map", [] {
        const LaserScan scan = asymmetricScan();
        const SlamOccupancyGrid grid = mappedGrid(scan);
        const std::uint64_t before = grid.getRevision();
        CorrelativeScanMatcher matcher(matcherConfig());
        const ScanMatchResult result = matcher.match(grid, scan, Pose2D{});
        return result.accepted && grid.getRevision() == before;
    });

    runTest(suite, "SLAM-MATCH-003", "full-circle selection is bounded and preserves sensor extrinsics", [] {
        LaserScan scan;
        scan.ranges = {70.0f, 80.0f, 90.0f, 100.0f, 65.0f, 75.0f, 85.0f, 95.0f};
        scan.angleMin = static_cast<float>(-kLocalizationPi);
        scan.angleIncrement = static_cast<float>(2.0 * kLocalizationPi / scan.ranges.size());
        scan.minRange = 1.0f;
        scan.maxRange = 150.0f;
        scan.sensorOffsetX = 12.0;
        scan.sensorOffsetY = -7.0;
        scan.sensorYawOffset = 0.2;
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.maximumBeams = 4;
        config.minimumUsableBeams = 4;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(grid, scan, Pose2D{});
        return result.accepted && result.selectedBeams == 4 && result.usedBeams == 4
            && result.score >= 0.99;
    });

    runTest(suite, "SLAM-MATCH-003", "sparse full-circle hits are selected from physical returns", [] {
        LaserScan scan;
        scan.ranges = {150.0f, 70.0f, 150.0f, 80.0f, 150.0f, 90.0f, 150.0f, 100.0f};
        scan.angleMin = static_cast<float>(-kLocalizationPi);
        scan.angleIncrement = static_cast<float>(2.0 * kLocalizationPi / scan.ranges.size());
        scan.minRange = 1.0f;
        scan.maxRange = 150.0f;
        const SlamOccupancyGrid grid = mappedGrid(scan);
        CorrelativeScanMatcherConfig config = matcherConfig();
        config.maximumBeams = 4;
        config.minimumUsableBeams = 4;
        CorrelativeScanMatcher matcher(config);
        const ScanMatchResult result = matcher.match(grid, scan, Pose2D{});
        return result.accepted && result.selectedBeams == 4
            && result.usedBeams == 4;
    });

    return suite.exitCode();
}
