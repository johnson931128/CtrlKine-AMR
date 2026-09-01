#include "localization/AmclLocalizer.hpp"
#include "sensors/LidarSimulator.hpp"
#include "localization/MapLikelihoodField.hpp"
#include "sensors/OdometrySimulator.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

#include "TestSupport.hpp"

namespace {
MapData makeLocalizationMap() {
    MapData map(50.0f);
    map.setWorldBoundary(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1000.0f, 800.0f)));
    for (int row = 1; row <= 7; ++row) {
        map.addObstacle(GridCoord{6, row});
    }
    for (int col = 7; col <= 15; ++col) {
        map.addObstacle(GridCoord{col, 10});
    }
    for (int col = 1; col <= 4; ++col) {
        map.addObstacle(GridCoord{col, 13});
    }
    map.addObstacle(GridCoord{14, 2});
    map.addObstacle(GridCoord{15, 2});
    map.addObstacle(GridCoord{16, 4});
    map.addObstacle(GridCoord{11, 6});
    map.addObstacle(GridCoord{18, 12});
    return map;
}

LidarSimulator makeLidar() {
    LidarConfig config;
    config.beamCount = 81;
    config.fieldOfView = 1.5 * kLocalizationPi;
    config.minRange = 1.0;
    config.maxRange = 900.0;
    config.rangeNoiseStdDev = 0.5;
    return LidarSimulator(config);
}

double positionError(const Pose2D& estimate, const Pose2D& truth) {
    return std::hypot(
        estimate.position.x - truth.position.x,
        estimate.position.y - truth.position.y
    );
}

double headingError(const Pose2D& estimate, const Pose2D& truth) {
    return std::abs(normalizeLocalizationAngle(estimate.heading - truth.heading));
}

bool finiteBelief(const AmclLocalizer& localizer, const AmclConfig& config) {
    const LocalizationEstimate& estimate = localizer.getEstimate();
    if (!estimate.valid || !std::isfinite(estimate.pose.position.x)
        || !std::isfinite(estimate.pose.position.y)
        || !std::isfinite(estimate.pose.heading)
        || estimate.particleCount < config.minParticles
        || estimate.particleCount > config.maxParticles) {
        return false;
    }
    double weightSum = 0.0;
    for (const Particle& particle : localizer.getParticles()) {
        if (!std::isfinite(particle.weight) || particle.weight < 0.0
            || !std::isfinite(particle.pose.position.x)
            || !std::isfinite(particle.pose.position.y)
            || !std::isfinite(particle.pose.heading)) {
            return false;
        }
        weightSum += particle.weight;
    }
    return std::isfinite(weightSum) && std::abs(weightSum - 1.0) < 1e-6;
}

bool applyScan(
    AmclLocalizer& localizer,
    const Pose2D& truth,
    const LidarSimulator& lidar,
    const MapLikelihoodField& field,
    const MapData& map,
    std::mt19937& randomEngine
) {
    localizer.forceSensorUpdate();
    const LaserScan scan = lidar.simulate(truth, map, randomEngine);
    return localizer.updateWithScan(scan, field, map, randomEngine);
}
}

int main() {
    TestSuite suite;

    runTest(suite, "INTEGRATION-CONVERGENCE", "meaningful local uncertainty converges on an asymmetric map", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 300;
        config.maxParticles = 1800;
        config.initialParticleCount = 1200;
        config.initialStdDevX = 180.0;
        config.initialStdDevY = 160.0;
        config.initialStdDevYaw = 0.9;
        config.sigmaHit = 18.0;
        config.maxBeams = 41;
        config.resampleInterval = 1;
        config.resampleEssRatio = 0.70;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        config.convergencePositionStdDev = 45.0;
        config.convergenceHeadingStdDev = 0.30;
        const Pose2D initialTruth{sf::Vector2f(170.0f, 215.0f), 0.37f};
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        OdometrySimulator odometry;
        std::mt19937 randomEngine(0x12345678u);
        odometry.reset(initialTruth);
        if (!localizer.initializeLocal(initialTruth, map, randomEngine)) {
            return false;
        }
        const LocalizationEstimate initial = localizer.getEstimate();
        const double initialCovariance = initial.covariance.xx() + initial.covariance.yy()
            + initial.covariance.yawYaw();

        bool allFinite = true;
        Pose2D truth = initialTruth;
        for (int update = 0; update < 18; ++update) {
            const double traveled = 5.0 * static_cast<double>(update + 1);
            truth.position = sf::Vector2f(
                initialTruth.position.x + static_cast<float>(traveled * std::cos(initialTruth.heading)),
                initialTruth.position.y + static_cast<float>(traveled * std::sin(initialTruth.heading))
            );
            truth.heading = initialTruth.heading + static_cast<float>(0.005 * (update + 1));
            localizer.accumulateOdometry(odometry.observe(truth, randomEngine));
            if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) {
                return false;
            }
            allFinite = allFinite && finiteBelief(localizer, config);
        }

        const LocalizationEstimate& estimate = localizer.getEstimate();
        const double finalCovariance = estimate.covariance.xx() + estimate.covariance.yy()
            + estimate.covariance.yawYaw();
        const double position = positionError(estimate.pose, truth);
        const double heading = headingError(estimate.pose, truth);
        if (!(allFinite && estimate.valid && position < 45.0 && heading < 0.30
              && finalCovariance < initialCovariance
              && localizer.getStatistics().state == LocalizationState::Converged
              && estimate.particleCount >= config.minParticles
              && estimate.particleCount <= config.maxParticles)) {
            std::cerr << "Convergence metrics: position=" << position
                      << " heading=" << heading
                      << " initialCov=" << initialCovariance
                      << " finalCov=" << finalCovariance
                      << " particles=" << estimate.particleCount << "\n";
            return false;
        }
        return true;
    });

    runTest(suite, "INTEGRATION-TRACKING", "laser updates outperform drifting raw odometry while moving", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 300;
        config.maxParticles = 1600;
        config.initialParticleCount = 1000;
        config.initialStdDevX = 100.0;
        config.initialStdDevY = 100.0;
        config.initialStdDevYaw = 0.55;
        config.sigmaHit = 18.0;
        config.maxBeams = 41;
        config.resampleInterval = 1;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        config.updateMinTranslation = 0.0;
        config.updateMinRotation = 0.0;
        const Pose2D start{sf::Vector2f(130.0f, 160.0f), 0.0f};

        OdometryConfig odometryConfig;
        odometryConfig.translationStdDevPerDistance = 0.08;
        odometryConfig.translationStdDevPerRotation = 1.0;
        odometryConfig.rotationStdDevPerRotation = 0.06;
        odometryConfig.rotationStdDevPerDistance = 0.0015;
        OdometrySimulator odometry(odometryConfig);
        odometry.reset(start);
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0xA55A1234u);
        if (!localizer.initializeLocal(start, map, randomEngine)) {
            return false;
        }
        for (int update = 0; update < 3; ++update) {
            applyScan(localizer, start, lidar, field, map, randomEngine);
        }

        std::vector<Pose2D> truthPath;
        for (int step = 1; step <= 6; ++step) {
            truthPath.push_back(Pose2D{sf::Vector2f(130.0f + 20.0f * step, 160.0f), 0.0f});
        }
        for (int step = 1; step <= 5; ++step) {
            truthPath.push_back(Pose2D{
                sf::Vector2f(250.0f, 160.0f),
                static_cast<float>((kLocalizationPi / 2.0) * step / 5.0)
            });
        }
        for (int step = 1; step <= 8; ++step) {
            truthPath.push_back(Pose2D{
                sf::Vector2f(250.0f, 160.0f + 25.0f * step),
                static_cast<float>(kLocalizationPi / 2.0)
            });
        }

        bool allFinite = true;
        bool estimateWasIndependent = false;
        for (const Pose2D& truth : truthPath) {
            const OdometryDelta delta = odometry.observe(truth, randomEngine);
            localizer.accumulateOdometry(delta);
            if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) {
                return false;
            }
            allFinite = allFinite && finiteBelief(localizer, config);
            estimateWasIndependent = estimateWasIndependent
                || positionError(localizer.getEstimate().pose, truth) > 0.01
                || headingError(localizer.getEstimate().pose, truth) > 0.001;
        }
        const Pose2D finalTruth = truthPath.back();
        for (int update = 0; update < 4; ++update) {
            applyScan(localizer, finalTruth, lidar, field, map, randomEngine);
        }

        const double odometryPositionError = positionError(odometry.getOdometryPose(), finalTruth);
        const double estimatePositionError = positionError(localizer.getEstimate().pose, finalTruth);
        const double estimateHeadingError = headingError(localizer.getEstimate().pose, finalTruth);
        if (!(allFinite && estimateWasIndependent && odometryPositionError > 5.0
              && estimatePositionError < odometryPositionError
              && estimatePositionError < 45.0 && estimateHeadingError < 0.30)) {
            std::cerr << "Tracking metrics: odomPosition=" << odometryPositionError
                      << " estimatePosition=" << estimatePositionError
                      << " estimateHeading=" << estimateHeadingError
                      << " particles=" << localizer.getEstimate().particleCount << "\n";
            return false;
        }
        return true;
    });

    runTest(suite, "INTEGRATION-KIDNAPPED", "recovery injection finds a distant kidnapped pose without filter reset", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 400;
        config.maxParticles = 2200;
        config.initialParticleCount = 1400;
        config.initialStdDevX = 70.0;
        config.initialStdDevY = 70.0;
        config.initialStdDevYaw = 0.45;
        config.sigmaHit = 16.0;
        config.maxBeams = 51;
        config.resampleInterval = 1;
        config.resampleEssRatio = 0.70;
        config.recoveryAlphaSlow = 0.02;
        config.recoveryAlphaFast = 0.55;
        config.convergencePositionStdDev = 50.0;
        config.convergenceHeadingStdDev = 0.35;
        const Pose2D poseA{sf::Vector2f(160.0f, 210.0f), 0.30f};
        const Pose2D poseB{sf::Vector2f(820.0f, 660.0f), -1.05f};

        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0xBADC0DEu);
        if (!localizer.initializeLocal(poseA, map, randomEngine)) {
            return false;
        }
        for (int update = 0; update < 12; ++update) {
            applyScan(localizer, poseA, lidar, field, map, randomEngine);
        }
        if (positionError(localizer.getEstimate().pose, poseA) >= 45.0
            || headingError(localizer.getEstimate().pose, poseA) >= 0.35
            || localizer.getStatistics().state != LocalizationState::Converged) {
            return false;
        }

        bool recoveryDetected = false;
        bool notDirectTruthCopy = false;
        bool allFinite = true;
        double maximumRecoveryProbability = 0.0;
        for (int update = 0; update < 60; ++update) {
            localizer.accumulateOdometry(OdometryDelta{});
            if (!applyScan(localizer, poseB, lidar, field, map, randomEngine)) {
                return false;
            }
            recoveryDetected = recoveryDetected
                || localizer.getStatistics().state == LocalizationState::Recovering
                || localizer.getStatistics().recoveryProbability > 0.05;
            maximumRecoveryProbability = std::max(
                maximumRecoveryProbability,
                localizer.getStatistics().recoveryProbability
            );
            if (update == 0) {
                notDirectTruthCopy = positionError(localizer.getEstimate().pose, poseB) > 100.0;
            }
            allFinite = allFinite && finiteBelief(localizer, config);
        }

        const double finalPositionError = positionError(localizer.getEstimate().pose, poseB);
        const double finalHeadingError = headingError(localizer.getEstimate().pose, poseB);
        if (!(recoveryDetected && notDirectTruthCopy && allFinite
              && finalPositionError < 55.0 && finalHeadingError < 0.40
              && localizer.getStatistics().state == LocalizationState::Converged
              && maximumRecoveryProbability > 0.05
              && localizer.getStatistics().recoveryProbability < maximumRecoveryProbability)) {
            std::cerr << "Kidnapped metrics: recovery=" << recoveryDetected
                      << " independent=" << notDirectTruthCopy
                      << " position=" << finalPositionError
                      << " heading=" << finalHeadingError
                      << " probability=" << localizer.getStatistics().recoveryProbability
                      << " particles=" << localizer.getEstimate().particleCount << "\n";
            return false;
        }
        return true;
    });

    runTest(suite, "INTEGRATION-NO-OBSTACLE", "open symmetric map remains finite without false convergence", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(800.0f, 800.0f)
        ));
        AmclConfig config;
        config.minParticles = 250;
        config.maxParticles = 1000;
        config.initialParticleCount = 800;
        config.maxBeams = 31;
        config.resampleInterval = 1;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0x1001u);
        const Pose2D truth{sf::Vector2f(250.0f, 250.0f), 0.0f};
        if (!localizer.initializeGlobal(map, randomEngine)) return false;
        for (int update = 0; update < 10; ++update) {
            if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) return false;
        }
        return finiteBelief(localizer, config)
            && localizer.getStatistics().support == LocalizationSupport::Insufficient
            && localizer.getStatistics().state != LocalizationState::Converged
            && !localizer.getEstimate().converged;
    });

    runTest(suite, "INTEGRATION-SYMMETRIC", "repeated-room belief is not presented as false convergence", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1200.0f, 500.0f)
        ));
        for (int roomOffset : {0, 10}) {
            for (int col = 1; col <= 8; ++col) {
                map.addObstacle(GridCoord{roomOffset + col, 1});
                map.addObstacle(GridCoord{roomOffset + col, 8});
            }
        }
        AmclConfig config;
        config.minParticles = 400;
        config.maxParticles = 2200;
        config.initialParticleCount = 2000;
        config.maxBeams = 41;
        config.resampleInterval = 1;
        config.clusterBinSizeX = 60.0;
        config.clusterBinSizeY = 60.0;
        config.clusterMinimumBinWeightRatio = 0.10;
        config.recoveryAlphaSlow = 0.01;
        config.recoveryAlphaFast = 0.20;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarConfig lidarConfig;
        lidarConfig.beamCount = 81;
        lidarConfig.fieldOfView = 1.5 * kLocalizationPi;
        lidarConfig.minRange = 1.0;
        lidarConfig.maxRange = 220.0;
        lidarConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator lidar(lidarConfig);
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0x2002u);
        Pose2D truth{sf::Vector2f(150.0f, 250.0f), 0.0f};
        if (!localizer.initializeGlobal(map, randomEngine)) return false;
        bool ambiguousSeen = false;
        for (int update = 0; update < 6; ++update) {
            if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) return false;
            ambiguousSeen = ambiguousSeen
                || localizer.getStatistics().state == LocalizationState::Ambiguous
                || localizer.getStatistics().significantClusterCount > 1;
        }
        const bool noFalseConvergence = finiteBelief(localizer, config) && ambiguousSeen
            && localizer.getStatistics().state != LocalizationState::Converged;
        for (int col = 11; col <= 18; ++col) {
            map.removeObstacle(GridCoord{col, 1});
            map.removeObstacle(GridCoord{col, 8});
        }
        for (int row = 2; row <= 7; ++row) map.addObstacle(GridCoord{6, row});
        for (int col = 7; col <= 12; ++col) map.addObstacle(GridCoord{col, 6});
        map.addObstacle(GridCoord{3, 3});
        map.addObstacle(GridCoord{4, 3});
        map.addObstacle(GridCoord{5, 4});
        map.addObstacle(GridCoord{2, 6});
        map.addObstacle(GridCoord{4, 4});
        map.addObstacle(GridCoord{7, 3});
        map.addObstacle(GridCoord{8, 7});
        map.addObstacle(GridCoord{11, 7});
        map.addObstacle(GridCoord{14, 2});
        map.addObstacle(GridCoord{15, 3});
        map.addObstacle(GridCoord{17, 5});
        map.addObstacle(GridCoord{20, 4});
        field.rebuildIfNeeded(map, config.likelihoodMaxDistance);
        localizer.forceSensorUpdate();
        LidarConfig richConfig;
        richConfig.beamCount = 121;
        richConfig.fieldOfView = 2.0 * kLocalizationPi;
        richConfig.minRange = 1.0;
        richConfig.maxRange = 900.0;
        richConfig.rangeNoiseStdDev = 0.0;
        LidarSimulator richLidar(richConfig);
        truth.heading = 0.37f;
        localizer.accumulateOdometry(OdometryDelta{0.0, 0.0, 0.37, true});
        for (int update = 0; update < 80; ++update) {
            if (update < 20) {
                truth.position.x += static_cast<float>(5.0 * std::cos(truth.heading));
                truth.position.y += static_cast<float>(5.0 * std::sin(truth.heading));
                localizer.accumulateOdometry(OdometryDelta{0.0, 5.0, 0.0, true});
            }
            if (!applyScan(localizer, truth, richLidar, field, map, randomEngine)) return false;
        }
        const bool dominantEmerged = localizer.getStatistics().dominantClusterWeight
                >= config.dominantClusterWeight
            && localizer.getStatistics().state != LocalizationState::Ambiguous
            && positionError(localizer.getEstimate().pose, truth) < 70.0
            && headingError(localizer.getEstimate().pose, truth) < 0.45;
        const bool passed = noFalseConvergence && dominantEmerged;
        if (!passed) {
            std::cerr << "Symmetric metrics: state="
                      << static_cast<int>(localizer.getStatistics().state)
                      << " clusters=" << localizer.getStatistics().clusterCount
                      << " significant=" << localizer.getStatistics().significantClusterCount
                      << " dominant=" << localizer.getStatistics().dominantClusterWeight
                      << " second=" << localizer.getStatistics().secondClusterWeight
                      << " ambiguousSeen=" << ambiguousSeen
                      << " error=" << positionError(localizer.getEstimate().pose, truth)
                      << " estimate=" << localizer.getEstimate().pose.position.x << ","
                      << localizer.getEstimate().pose.position.y
                      << " yaw=" << localizer.getEstimate().pose.heading << "\n";
        }
        return passed;
    });

    runTest(suite, "INTEGRATION-MAP-EDIT", "geometry edit rebuilds field and continues tracking", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 200;
        config.maxParticles = 800;
        config.initialParticleCount = 600;
        config.maxBeams = 31;
        config.resampleInterval = 1;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        const std::uint64_t beforeRevision = field.getSourceRevision();
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0x3003u);
        const Pose2D truth{sf::Vector2f(170.0f, 215.0f), 0.37f};
        if (!localizer.initializeLocal(truth, map, randomEngine)) return false;
        if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) return false;
        map.addObstacle(GridCoord{17, 3});
        field.rebuildIfNeeded(map, config.likelihoodMaxDistance);
        localizer.forceSensorUpdate();
        const bool updated = localizer.updateWithScan(
            lidar.simulate(truth, map, randomEngine), field, map, randomEngine
        );
        return updated && field.getSourceRevision() > beforeRevision
            && finiteBelief(localizer, config);
    });

    runTest(suite, "INTEGRATION-NEAR-BOUNDARY", "near-boundary scan and localization remain finite", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 200;
        config.maxParticles = 600;
        config.initialParticleCount = 400;
        config.initialStdDevX = 20.0;
        config.initialStdDevY = 20.0;
        config.initialStdDevYaw = 0.2;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0x4004u);
        const Pose2D truth{sf::Vector2f(55.0f, 55.0f), 0.2f};
        if (!localizer.initializeLocal(truth, map, randomEngine)) return false;
        for (int update = 0; update < 4; ++update) {
            if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) return false;
        }
        return finiteBelief(localizer, config);
    });

    runTest(suite, "INTEGRATION-GLOBAL", "global localization forms a supported dominant hypothesis", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 400;
        config.maxParticles = 2600;
        config.initialParticleCount = 2200;
        config.sigmaHit = 18.0;
        config.maxBeams = 51;
        config.resampleInterval = 1;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        config.convergencePositionStdDev = 55.0;
        config.convergenceHeadingStdDev = 0.40;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarSimulator lidar = makeLidar();
        AmclLocalizer localizer(config);
        std::mt19937 randomEngine(0x5005u);
        Pose2D truth{sf::Vector2f(170.0f, 215.0f), 0.37f};
        if (!localizer.initializeGlobal(map, randomEngine)) return false;
        bool falseEarlyConvergence = false;
        for (int update = 0; update < 30; ++update) {
            if (update == 15) {
                const double turn = kLocalizationPi / 2.0;
                truth.heading = static_cast<float>(normalizeLocalizationAngle(
                    truth.heading + turn
                ));
                localizer.accumulateOdometry(OdometryDelta{0.0, 0.0, turn, true});
            } else {
                truth.position.x += static_cast<float>(5.0 * std::cos(truth.heading));
                truth.position.y += static_cast<float>(5.0 * std::sin(truth.heading));
                localizer.accumulateOdometry(OdometryDelta{0.0, 5.0, 0.0, true});
            }
            if (!applyScan(localizer, truth, lidar, field, map, randomEngine)) return false;
            if (localizer.getStatistics().state == LocalizationState::Converged) {
                const double currentPosition = positionError(localizer.getEstimate().pose, truth);
                const double currentHeading = headingError(localizer.getEstimate().pose, truth);
                if (update + 1 < static_cast<int>(config.minimumGlobalSensorUpdatesForConvergence)
                    || currentPosition >= 125.0 || currentHeading >= 1.25
                    || localizer.getStatistics().support != LocalizationSupport::Good
                    || localizer.getStatistics().significantClusterCount > 1) {
                    falseEarlyConvergence = true;
                }
            }
        }
        const double position = positionError(localizer.getEstimate().pose, truth);
        const double heading = headingError(localizer.getEstimate().pose, truth);
        const bool passed = !falseEarlyConvergence && finiteBelief(localizer, config)
            && localizer.getStatistics().state == LocalizationState::Converged
            && localizer.getStatistics().dominantClusterWeight >= config.dominantClusterWeight
            && position < 70.0 && heading < 0.45;
        if (!passed) {
            std::cerr << "Global metrics: state=" << static_cast<int>(localizer.getStatistics().state)
                      << " position=" << position << " heading=" << heading
                      << " dominant=" << localizer.getStatistics().dominantClusterWeight
                      << " second=" << localizer.getStatistics().secondClusterWeight
                      << " particles=" << localizer.getEstimate().particleCount << "\n";
        }
        return passed;
    });

    runTest(suite, "INTEGRATION-CORRIDOR", "corridor covariance retains longitudinal uncertainty", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1200.0f, 500.0f)
        ));
        for (int col = 1; col <= 22; ++col) {
            map.addObstacle(GridCoord{col, 1});
            map.addObstacle(GridCoord{col, 8});
        }
        AmclConfig config;
        config.minParticles = 300;
        config.maxParticles = 2200;
        config.initialParticleCount = 2000;
        config.initialStdDevX = 140.0;
        config.initialStdDevY = 140.0;
        config.initialStdDevYaw = 0.35;
        config.maxBeams = 41;
        config.resampleInterval = 100;
        config.clusterBinSizeX = 100.0;
        config.clusterBinSizeY = 100.0;
        config.clusterMinimumBinWeightRatio = 0.01;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        LidarConfig corridorLidar;
        corridorLidar.beamCount = 81;
        corridorLidar.fieldOfView = 1.5 * kLocalizationPi;
        corridorLidar.minRange = 1.0;
        corridorLidar.maxRange = 220.0;
        corridorLidar.rangeNoiseStdDev = 0.0;
        LidarSimulator sensor(corridorLidar);
        AmclLocalizer localizer(config);
        std::mt19937 engine(0x6006u);
        const Pose2D truth{sf::Vector2f(500.0f, 250.0f), 0.0f};
        if (!localizer.initializeLocal(truth, map, engine)) return false;
        const LocalizationCovariance initialCovariance = localizer.getEstimate().covariance;
        for (int update = 0; update < 10; ++update) {
            if (!applyScan(localizer, truth, sensor, field, map, engine)) return false;
        }
        const LocalizationCovariance& covariance = localizer.getEstimate().covariance;
        const bool passed = finiteBelief(localizer, config)
            && covariance.xx() > covariance.yy() * 2.0
            && covariance.yy() < initialCovariance.yy() * 0.5
            && covariance.xx() > initialCovariance.xx() * 0.3
            && localizer.getStatistics().state != LocalizationState::Converged;
        if (!passed) {
            std::cerr << "Corridor metrics: xx=" << covariance.xx()
                      << " yy=" << covariance.yy()
                      << " state=" << static_cast<int>(localizer.getStatistics().state)
                      << " support=" << static_cast<int>(localizer.getStatistics().support)
                      << " clusters=" << localizer.getStatistics().significantClusterCount << "\n";
        }
        return passed;
    });

    runTest(suite, "INTEGRATION-LIFECYCLE", "repeated local, global, failed, and seeded resets leave no stale belief", [] {
        MapData map = makeLocalizationMap();
        AmclConfig config;
        config.minParticles = 100;
        config.maxParticles = 300;
        config.initialParticleCount = 200;
        AmclLocalizer localizer(config);
        const Pose2D truth{sf::Vector2f(170.0f, 215.0f), 0.37f};
        std::mt19937 engine(0x7777u);
        if (!localizer.initializeLocal(truth, map, engine)) return false;
        localizer.appendHistory(truth);
        if (localizer.getHistory().empty() || !localizer.initializeGlobal(map, engine)) return false;
        if (!localizer.getHistory().empty()
            || localizer.getStatistics().initialization != LocalizationInitialization::Global) {
            return false;
        }

        MapData blocked(50.0f);
        blocked.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(50.0f, 50.0f)
        ));
        blocked.addObstacle(GridCoord{0, 0});
        if (localizer.initializeLocal(
                Pose2D{sf::Vector2f(25.0f, 25.0f), 0.0f}, blocked, engine)) return false;
        if (!localizer.getParticles().empty() || localizer.getEstimate().valid
            || localizer.getStatistics().state != LocalizationState::Uninitialized) return false;

        std::mt19937 firstSeed(12345u);
        std::mt19937 secondSeed(12345u);
        AmclLocalizer first(config);
        AmclLocalizer second(config);
        if (!first.initializeLocal(truth, map, firstSeed)
            || !second.initializeLocal(truth, map, secondSeed)) return false;
        return std::equal(
            first.getParticles().begin(), first.getParticles().end(),
            second.getParticles().begin(), [](const Particle& a, const Particle& b) {
                return a.pose.position == b.pose.position
                    && a.pose.heading == b.pose.heading && a.weight == b.weight;
            }
        );
    });

    return suite.exitCode();
}
