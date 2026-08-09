#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "CoordinateTypes.hpp"

constexpr double kLocalizationPi = 3.14159265358979323846;

inline double normalizeLocalizationAngle(double angle) {
    while (angle < -kLocalizationPi) {
        angle += 2.0 * kLocalizationPi;
    }
    while (angle >= kLocalizationPi) {
        angle -= 2.0 * kLocalizationPi;
    }
    return angle;
}

struct Particle {
    Pose2D pose;
    double weight = 0.0;
};

struct LaserScan {
    std::vector<float> ranges;
    float angleMin = 0.0f;
    float angleIncrement = 0.0f;
    float minRange = 0.0f;
    float maxRange = 0.0f;
};

struct OdometryDelta {
    double rotation1 = 0.0;
    double translation = 0.0;
    double rotation2 = 0.0;
    bool valid = true;
};

struct SensorUpdateResult {
    bool updated = false;
    double observationQuality = 0.0;
};

struct LocalizationCovariance {
    std::array<double, 9> values{};

    double xx() const { return values[0]; }
    double yy() const { return values[4]; }
    double yawYaw() const { return values[8]; }
};

enum class LocalizationState {
    Uninitialized,
    Tracking,
    Converged,
    Recovering
};

struct LocalizationEstimate {
    Pose2D pose;
    LocalizationCovariance covariance;
    bool valid = false;
    bool converged = false;
    std::size_t particleCount = 0;
    double effectiveSampleSize = 0.0;
};

struct LocalizationStatistics {
    LocalizationState state = LocalizationState::Uninitialized;
    std::size_t particleCount = 0;
    double effectiveSampleSize = 0.0;
    double slowWeightAverage = 0.0;
    double fastWeightAverage = 0.0;
    double recoveryProbability = 0.0;
    std::size_t sensorUpdateCount = 0;
};

struct LidarConfig {
    std::size_t beamCount = 91;
    double fieldOfView = 1.5 * kLocalizationPi;
    double minRange = 5.0;
    double maxRange = 800.0;
    double sensorHeading = 0.0;
    double rangeNoiseStdDev = 1.0;
};

struct OdometryConfig {
    double translationStdDevPerDistance = 0.01;
    double translationStdDevPerRotation = 0.25;
    double rotationStdDevPerRotation = 0.01;
    double rotationStdDevPerDistance = 0.0005;
};

struct AmclConfig {
    std::size_t minParticles = 300;
    std::size_t maxParticles = 2000;
    std::size_t initialParticleCount = 800;

    double alpha1 = 0.02;
    double alpha2 = 0.00001;
    double alpha3 = 0.0025;
    double alpha4 = 0.05;
    double alpha5 = 0.0004;

    double initialStdDevX = 75.0;
    double initialStdDevY = 75.0;
    double initialStdDevYaw = 0.45;

    double sigmaHit = 25.0;
    double zHit = 0.95;
    double zRand = 0.05;
    double likelihoodMaxDistance = 150.0;
    std::size_t maxBeams = 31;

    double updateMinTranslation = 10.0;
    double updateMinRotation = 0.10;
    std::size_t resampleInterval = 2;
    double resampleEssRatio = 0.50;

    double kldBinSizeX = 50.0;
    double kldBinSizeY = 50.0;
    double kldBinSizeYaw = 0.17453292519943295;
    double pfErr = 0.05;
    double pfZ = 2.3263478740408408;

    double recoveryAlphaSlow = 0.001;
    double recoveryAlphaFast = 0.10;

    double convergencePositionStdDev = 35.0;
    double convergenceHeadingStdDev = 0.25;

    std::uint32_t randomSeed = 0x00C0FFEEu;
};
