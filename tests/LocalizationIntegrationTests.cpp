#include "AmclLocalizer.hpp"
#include "LidarSimulator.hpp"
#include "MapLikelihoodField.hpp"
#include "OdometrySimulator.hpp"

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
        for (int update = 0; update < 60; ++update) {
            localizer.accumulateOdometry(OdometryDelta{});
            if (!applyScan(localizer, poseB, lidar, field, map, randomEngine)) {
                return false;
            }
            recoveryDetected = recoveryDetected
                || localizer.getStatistics().state == LocalizationState::Recovering
                || localizer.getStatistics().recoveryProbability > 0.05;
            if (update == 0) {
                notDirectTruthCopy = positionError(localizer.getEstimate().pose, poseB) > 100.0;
            }
            allFinite = allFinite && finiteBelief(localizer, config);
        }

        const double finalPositionError = positionError(localizer.getEstimate().pose, poseB);
        const double finalHeadingError = headingError(localizer.getEstimate().pose, poseB);
        if (!(recoveryDetected && notDirectTruthCopy && allFinite
              && finalPositionError < 55.0 && finalHeadingError < 0.40
              && localizer.getStatistics().state == LocalizationState::Converged)) {
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

    return suite.exitCode();
}
