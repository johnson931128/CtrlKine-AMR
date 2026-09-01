#include "SlamTestUtils.hpp"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>

#include "TestSupport.hpp"

namespace {
struct Aggregate {
    std::size_t successes = 0;
    double meanPositionRmse = 0.0;
    double worstPositionRmse = 0.0;
    double meanHeadingRmse = 0.0;
    double worstHeadingRmse = 0.0;
    double meanIou = 0.0;
    double worstIou = 1.0;
    double meanAgreement = 0.0;
    double worstAgreement = 1.0;
    double meanCoverage = 0.0;
    double worstCoverage = 1.0;
};

Aggregate runSeeds(
    const char* label,
    const OdometryConfig& odometry,
    double lidarNoise,
    const std::vector<std::uint32_t>& seeds,
    bool rotatedCorridor = false
) {
    Aggregate aggregate;
    std::cout << std::fixed << std::setprecision(4);
    for (const std::uint32_t seed : seeds) {
        const SlamScenarioMetrics metrics = rotatedCorridor
            ? runRotatedCorridorScenario(seed, odometry, lidarNoise)
            : runSlamScenario(seed, odometry, lidarNoise);
        aggregate.successes += metrics.completed ? 1 : 0;
        const double gatedPositionRmse = metrics.completed
            ? metrics.positionRmse : std::max(100.0, metrics.positionRmse);
        const double gatedHeadingRmse = metrics.completed
            ? metrics.headingRmseRadians
            : std::max(kLocalizationPi, metrics.headingRmseRadians);
        aggregate.meanPositionRmse += gatedPositionRmse;
        aggregate.worstPositionRmse = std::max(
            aggregate.worstPositionRmse, gatedPositionRmse
        );
        aggregate.meanHeadingRmse += gatedHeadingRmse;
        aggregate.worstHeadingRmse = std::max(
            aggregate.worstHeadingRmse, gatedHeadingRmse
        );
        aggregate.meanIou += metrics.occupiedIou;
        aggregate.worstIou = std::min(aggregate.worstIou, metrics.occupiedIou);
        aggregate.meanAgreement += metrics.occupancyAgreement;
        aggregate.worstAgreement = std::min(
            aggregate.worstAgreement, metrics.occupancyAgreement
        );
        aggregate.meanCoverage += metrics.knownCoverage;
        aggregate.worstCoverage = std::min(
            aggregate.worstCoverage, metrics.knownCoverage
        );
        std::cout << label << " seed " << seed
                  << ": complete=" << (metrics.completed ? "yes" : "no")
                  << ", lost=" << metrics.lostFrames
                  << ", missing=" << metrics.missingPoseFrames
                  << ", posRMSE=" << metrics.positionRmse
                  << ", odomRMSE=" << metrics.odometryPositionRmse
                  << ", yawRMSEdeg=" << metrics.headingRmseRadians * 57.2957795
                  << ", accepted/rejected=" << metrics.acceptedMatches
                  << "/" << metrics.rejectedMatches
                  << ", IoU=" << metrics.occupiedIou
                  << ", agreement=" << metrics.occupancyAgreement
                  << ", coverage=" << metrics.knownCoverage << "\n";
    }
    const double count = static_cast<double>(seeds.size());
    aggregate.meanPositionRmse /= count;
    aggregate.meanHeadingRmse /= count;
    aggregate.meanIou /= count;
    aggregate.meanAgreement /= count;
    aggregate.meanCoverage /= count;
    std::cout << label << " aggregate: success=" << aggregate.successes << "/"
              << seeds.size() << ", mean/worst posRMSE="
              << aggregate.meanPositionRmse << "/" << aggregate.worstPositionRmse
              << ", mean/worst yawRMSEdeg="
              << aggregate.meanHeadingRmse * 57.2957795 << "/"
              << aggregate.worstHeadingRmse * 57.2957795
              << ", mean/worst IoU=" << aggregate.meanIou << "/"
              << aggregate.worstIou << ", agreement=" << aggregate.meanAgreement
              << "/" << aggregate.worstAgreement
              << ", coverage=" << aggregate.meanCoverage << "/"
              << aggregate.worstCoverage << "\n";
    return aggregate;
}
}

int main() {
    TestSuite suite;
    const std::vector<std::uint32_t> seeds{101, 202, 303, 404, 505};

    runTest(suite, "SLAM-STRESS-IDEAL", "ideal trajectory has near-zero error and strong mapping agreement", [] {
        OdometryConfig odometry;
        odometry.translationStdDevPerDistance = 0.0;
        odometry.translationStdDevPerRotation = 0.0;
        odometry.rotationStdDevPerRotation = 0.0;
        odometry.rotationStdDevPerDistance = 0.0;
        const SlamScenarioMetrics metrics = runSlamScenario(1, odometry, 0.0);
        return metrics.completed && metrics.positionRmse <= 1.0
            && metrics.headingRmseRadians <= 0.01
            && metrics.occupiedIou >= 0.70
            && metrics.occupancyAgreement >= 0.98
            && metrics.knownCoverage >= 0.25;
    });

    runTest(suite, "SLAM-STRESS-NOMINAL", "five nominal sensor-noise seeds track within calibrated gates", [&] {
        const Aggregate aggregate = runSeeds(
            "nominal", OdometryConfig{}, 1.0, seeds
        );
        return aggregate.successes == seeds.size()
            && aggregate.meanPositionRmse <= 12.0
            && aggregate.worstPositionRmse <= 20.0
            && aggregate.meanHeadingRmse <= 0.08
            && aggregate.worstHeadingRmse <= 0.11
            && aggregate.meanIou >= 0.20
            && aggregate.worstIou >= 0.09
            && aggregate.meanAgreement >= 0.95
            && aggregate.worstAgreement >= 0.97
            && aggregate.meanCoverage >= 0.34
            && aggregate.worstCoverage >= 0.34;
    });

    runTest(suite, "SLAM-STRESS-ELEVATED", "five elevated sensor-noise seeds remain bounded", [&] {
        OdometryConfig elevated;
        elevated.translationStdDevPerDistance *= 2.0;
        elevated.translationStdDevPerRotation *= 2.0;
        elevated.rotationStdDevPerRotation *= 2.0;
        elevated.rotationStdDevPerDistance *= 2.0;
        const Aggregate aggregate = runSeeds("elevated", elevated, 3.0, seeds);
        return aggregate.successes >= 4
            && aggregate.meanPositionRmse <= 20.0
            && aggregate.worstPositionRmse <= 35.0
            && aggregate.meanHeadingRmse <= 0.15
            && aggregate.worstHeadingRmse <= 0.12
            && aggregate.meanIou >= 0.25
            && aggregate.worstIou >= 0.12
            && aggregate.meanAgreement >= 0.92
            && aggregate.worstAgreement >= 0.96
            && aggregate.meanCoverage >= 0.34
            && aggregate.worstCoverage >= 0.34;
    });

    runTest(suite, "SLAM-STRESS-ROTATED-CORRIDOR", "rotated-frame corridor trajectory tracks across three nominal seeds", [&] {
        const std::vector<std::uint32_t> corridorSeeds{606, 707, 808};
        const Aggregate aggregate = runSeeds(
            "rotated-corridor", OdometryConfig{}, 1.0, corridorSeeds, true
        );
        return aggregate.successes == corridorSeeds.size()
            && aggregate.meanPositionRmse <= 15.0
            && aggregate.worstPositionRmse <= 25.0
            && aggregate.worstHeadingRmse <= 0.12
            && aggregate.meanAgreement >= 0.95
            && aggregate.worstCoverage >= 0.20;
    });

    runTest(suite, "SLAM-STRESS-SAFETY", "no-feature bootstrap and large jump fail safely without mapping", [] {
        SlamFrontend frontend(makeSlamScenarioConfig());
        LaserScan allMax;
        allMax.ranges.assign(91, 200.0f);
        allMax.angleMin = static_cast<float>(-kLocalizationPi);
        allMax.angleIncrement = static_cast<float>(2.0 * kLocalizationPi / 91.0);
        allMax.minRange = 1.0f;
        allMax.maxRange = 200.0f;
        const SlamUpdateResult empty = frontend.process(OdometryDelta{}, allMax);
        if (empty.state != SlamState::Uninitialized || frontend.getMap().getRevision() != 0) {
            return false;
        }
        MapData map = makeSlamTestMap();
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 91;
        lidarConfig.fieldOfView = 2.0 * kLocalizationPi;
        lidarConfig.minRange = 1.0;
        lidarConfig.maxRange = 200.0;
        lidarConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(lidarConfig);
        std::mt19937 rng(8);
        const LaserScan scan = lidar.simulate(
            Pose2D{sf::Vector2f(-120.0f, -100.0f), 0.0f}, map, rng
        );
        frontend.process(OdometryDelta{}, scan);
        const std::uint64_t revision = frontend.getMap().getRevision();
        const SlamUpdateResult jump = frontend.process(
            OdometryDelta{0.0, 500.0, 0.0, true}, scan
        );
        return jump.state == SlamState::Lost && !jump.mapIntegrated
            && frontend.getMap().getRevision() == revision;
    });

    return suite.exitCode();
}
