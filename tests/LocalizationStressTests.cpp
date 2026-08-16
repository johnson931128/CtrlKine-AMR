#include "AmclLocalizer.hpp"
#include "LidarSimulator.hpp"
#include "MapLikelihoodField.hpp"

#include <cmath>
#include <iostream>
#include <random>

#include "TestSupport.hpp"

namespace {
MapData featureMap() {
    MapData map(50.0f);
    map.setWorldBoundary(sf::FloatRect(
        sf::Vector2f(0.0f, 0.0f), sf::Vector2f(1000.0f, 800.0f)
    ));
    for (int row = 1; row <= 7; ++row) map.addObstacle(GridCoord{6, row});
    for (int col = 7; col <= 15; ++col) map.addObstacle(GridCoord{col, 10});
    for (int col = 1; col <= 4; ++col) map.addObstacle(GridCoord{col, 13});
    for (const GridCoord cell : {
            GridCoord{2, 2}, GridCoord{3, 2}, GridCoord{2, 5}, GridCoord{4, 4},
            GridCoord{4, 5}, GridCoord{9, 2}, GridCoord{14, 2},
            GridCoord{16, 4}, GridCoord{11, 6}, GridCoord{17, 7}, GridCoord{2, 10},
            GridCoord{12, 13}, GridCoord{18, 12}
        }) {
        map.addObstacle(cell);
    }
    return map;
}

LidarSimulator lidar() {
    LidarConfig config;
    config.beamCount = 61;
    config.fieldOfView = 2.0 * kLocalizationPi;
    config.minRange = 1.0;
    config.maxRange = 900.0;
    config.rangeNoiseStdDev = 0.5;
    return LidarSimulator(config);
}

double error(const Pose2D& first, const Pose2D& second) {
    return std::hypot(
        first.position.x - second.position.x,
        first.position.y - second.position.y
    );
}

bool scan(
    AmclLocalizer& localizer,
    const Pose2D& truth,
    const LidarSimulator& sensor,
    const MapLikelihoodField& field,
    const MapData& map,
    std::mt19937& engine
) {
    localizer.forceSensorUpdate();
    return localizer.updateWithScan(sensor.simulate(truth, map, engine), field, map, engine);
}
}

int main() {
    TestSuite suite;

    runTest(suite, "STRESS-LOCAL", "feature-rich local localization succeeds across 10 seeds", [] {
        const MapData map = featureMap();
        AmclConfig config;
        config.minParticles = 250;
        config.maxParticles = 1200;
        config.initialParticleCount = 900;
        config.initialStdDevX = 140.0;
        config.initialStdDevY = 120.0;
        config.initialStdDevYaw = 0.7;
        config.sigmaHit = 18.0;
        config.maxBeams = 41;
        config.resampleInterval = 1;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        const LidarSimulator sensor = lidar();
        const Pose2D truth{sf::Vector2f(170.0f, 215.0f), 0.37f};
        int successes = 0;
        double finalErrorSum = 0.0;
        for (std::uint32_t seed = 1; seed <= 10; ++seed) {
            std::mt19937 engine(seed);
            AmclLocalizer localizer(config);
            if (!localizer.initializeLocal(truth, map, engine)) continue;
            bool okay = true;
            for (int update = 0; update < 16; ++update) {
                okay = okay && scan(localizer, truth, sensor, field, map, engine);
            }
            const double finalError = error(localizer.getEstimate().pose, truth);
            finalErrorSum += finalError;
            if (okay && finalError < 55.0
                && localizer.getStatistics().state == LocalizationState::Converged) {
                ++successes;
            }
        }
        std::cout << "Local stress: successes=" << successes
                  << "/10 meanFinalError=" << finalErrorSum / 10.0 << "\n";
        return successes == 10;
    });

    runTest(suite, "STRESS-GLOBAL", "feature-rich global localization is bounded across 10 seeds", [] {
        const MapData map = featureMap();
        AmclConfig config;
        config.minParticles = 350;
        config.maxParticles = 2200;
        config.initialParticleCount = 1800;
        config.sigmaHit = 18.0;
        config.maxBeams = 41;
        config.resampleInterval = 1;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        const LidarSimulator sensor = lidar();
        int successes = 0;
        int falseConvergence = 0;
        for (std::uint32_t seed = 101; seed <= 110; ++seed) {
            std::mt19937 engine(seed);
            AmclLocalizer localizer(config);
            Pose2D truth{sf::Vector2f(170.0f, 215.0f), 0.37f};
            if (!localizer.initializeGlobal(map, engine)) continue;
            bool okay = true;
            for (int update = 0; update < 28; ++update) {
                if (update >= 8 && update < 14) {
                    const double turn = kLocalizationPi / 12.0;
                    truth.heading = static_cast<float>(normalizeLocalizationAngle(
                        truth.heading + turn
                    ));
                    localizer.accumulateOdometry(OdometryDelta{0.0, 0.0, turn, true});
                } else {
                    truth.position.x += static_cast<float>(4.0 * std::cos(truth.heading));
                    truth.position.y += static_cast<float>(4.0 * std::sin(truth.heading));
                    localizer.accumulateOdometry(OdometryDelta{0.0, 4.0, 0.0, true});
            }
            okay = okay && scan(localizer, truth, sensor, field, map, engine);
                if (localizer.getStatistics().state == LocalizationState::Converged) {
                    const double currentPosition = error(localizer.getEstimate().pose, truth);
                    const double currentHeading = std::abs(normalizeLocalizationAngle(
                        localizer.getEstimate().pose.heading - truth.heading
                    ));
                    if (update + 1 < static_cast<int>(config.minimumGlobalSensorUpdatesForConvergence)
                        || currentPosition >= 125.0 || currentHeading >= 1.25
                        || localizer.getStatistics().support != LocalizationSupport::Good
                        || localizer.getStatistics().significantClusterCount > 1) {
                        ++falseConvergence;
                    }
                }
            }
            const double finalError = error(localizer.getEstimate().pose, truth);
            const double finalHeadingError = std::abs(normalizeLocalizationAngle(
                localizer.getEstimate().pose.heading - truth.heading
            ));
            const bool withinDocumentedGlobalBound = finalError < 125.0
                && finalHeadingError < 1.25;
            if (localizer.getStatistics().state == LocalizationState::Converged
                && !withinDocumentedGlobalBound) {
                ++falseConvergence;
            }
            if (okay && withinDocumentedGlobalBound
                && localizer.getStatistics().state == LocalizationState::Converged) {
                ++successes;
            } else {
                std::cout << " globalSeed=" << seed
                          << " state=" << static_cast<int>(localizer.getStatistics().state)
                          << " error=" << finalError
                          << " estimate=(" << localizer.getEstimate().pose.position.x
                          << "," << localizer.getEstimate().pose.position.y
                          << "," << localizer.getEstimate().pose.heading << ")"
                          << " truth=(" << truth.position.x << "," << truth.position.y
                          << "," << truth.heading << ")"
                          << " dominant=" << localizer.getStatistics().dominantClusterWeight
                          << " second=" << localizer.getStatistics().secondClusterWeight
                          << " particles=" << localizer.getEstimate().particleCount << "\n";
                std::cout << "  observationQuality="
                          << localizer.getStatistics().sensor.observationQuality << "\n";
            }
        }
        std::cout << "Global stress: successes=" << successes
                  << "/10 falseConvergence=" << falseConvergence << "\n";
        return successes >= 9 && falseConvergence == 0;
    });

    runTest(suite, "STRESS-NO-FEATURE", "open maps produce zero false convergence across 10 seeds", [] {
        MapData map(50.0f);
        map.setWorldBoundary(sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f), sf::Vector2f(800.0f, 800.0f)
        ));
        AmclConfig config;
        config.minParticles = 200;
        config.maxParticles = 700;
        config.initialParticleCount = 600;
        config.resampleInterval = 1;
        config.recoveryAlphaSlow = 0.0;
        config.recoveryAlphaFast = 0.0;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        const LidarSimulator sensor = lidar();
        int falseConvergence = 0;
        for (std::uint32_t seed = 201; seed <= 210; ++seed) {
            std::mt19937 engine(seed);
            AmclLocalizer localizer(config);
            const Pose2D truth{sf::Vector2f(250.0f, 250.0f), 0.0f};
            if (!localizer.initializeGlobal(map, engine)) return false;
            for (int update = 0; update < 8; ++update) {
                if (!scan(localizer, truth, sensor, field, map, engine)) return false;
                if (localizer.getStatistics().state == LocalizationState::Converged) {
                    ++falseConvergence;
                }
            }
        }
        std::cout << "No-feature stress: falseConvergence=" << falseConvergence << "\n";
        return falseConvergence == 0;
    });

    runTest(suite, "STRESS-KIDNAPPED", "kidnapped recovery succeeds across 5 deterministic seeds", [] {
        const MapData map = featureMap();
        AmclConfig config;
        config.minParticles = 350;
        config.maxParticles = 2200;
        config.initialParticleCount = 1300;
        config.initialStdDevX = 70.0;
        config.initialStdDevY = 70.0;
        config.initialStdDevYaw = 0.45;
        config.sigmaHit = 16.0;
        config.maxBeams = 51;
        config.resampleInterval = 1;
        config.recoveryAlphaSlow = 0.02;
        config.recoveryAlphaFast = 0.55;
        MapLikelihoodField field;
        field.rebuild(map, config.likelihoodMaxDistance);
        const LidarSimulator sensor = lidar();
        const Pose2D poseA{sf::Vector2f(170.0f, 215.0f), 0.30f};
        const Pose2D poseB{sf::Vector2f(820.0f, 660.0f), -1.05f};
        int successes = 0;
        for (std::uint32_t seed = 301; seed <= 305; ++seed) {
            std::mt19937 engine(seed);
            AmclLocalizer localizer(config);
            if (!localizer.initializeLocal(poseA, map, engine)) continue;
            for (int update = 0; update < 12; ++update) {
                if (!scan(localizer, poseA, sensor, field, map, engine)) return false;
            }
            bool recoverySeen = false;
            for (int update = 0; update < 60; ++update) {
                if (!scan(localizer, poseB, sensor, field, map, engine)) return false;
                recoverySeen = recoverySeen
                    || localizer.getStatistics().state == LocalizationState::Recovering
                    || localizer.getStatistics().recoveryProbability > 0.05;
            }
            if (recoverySeen && error(localizer.getEstimate().pose, poseB) < 70.0
                && localizer.getStatistics().state == LocalizationState::Converged) {
                ++successes;
            }
        }
        std::cout << "Kidnapped stress: successes=" << successes << "/5\n";
        return successes == 5;
    });

    return suite.exitCode();
}
