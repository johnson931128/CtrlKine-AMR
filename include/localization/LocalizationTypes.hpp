#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "core/CoordinateTypes.hpp"

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
    double sensorOffsetX = 0.0;
    double sensorOffsetY = 0.0;
    double sensorYawOffset = 0.0;
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
    double likelihoodContrast = 0.0;
    std::size_t totalBeams = 0;
    std::size_t selectedBeams = 0;
    std::size_t usedBeams = 0;
    std::size_t skippedBeams = 0;
    std::size_t invalidBeams = 0;
    std::size_t maxRangeBeams = 0;
    bool beamSkipFallback = false;
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
    Ambiguous,
    Converged,
    Recovering
};

enum class LocalizationSupport {
    Insufficient,
    Weak,
    Good
};

enum class LocalizationInitialization {
    None,
    Local,
    Global
};

struct ParticleCluster {
    std::size_t particleCount = 0;
    double weight = 0.0;
    Pose2D pose;
    LocalizationCovariance covariance;
    sf::FloatRect spatialExtent;
    double headingExtent = 0.0;
    double headingResultant = 0.0;
};

struct LocalizationEstimate {
    Pose2D pose;
    LocalizationCovariance covariance;
    bool valid = false;
    bool converged = false;
    std::size_t particleCount = 0;
    double effectiveSampleSize = 0.0;
    double dominantWeight = 0.0;
    double secondWeight = 0.0;
    std::size_t significantClusterCount = 0;
};

struct LocalizationStatistics {
    LocalizationState state = LocalizationState::Uninitialized;
    std::size_t particleCount = 0;
    double effectiveSampleSize = 0.0;
    double preResampleEffectiveSampleSize = 0.0;
    double slowWeightAverage = 0.0;
    double fastWeightAverage = 0.0;
    double recoveryProbability = 0.0;
    std::size_t sensorUpdateCount = 0;
    LocalizationSupport support = LocalizationSupport::Insufficient;
    LocalizationInitialization initialization = LocalizationInitialization::None;
    double dominantClusterWeight = 0.0;
    double secondClusterWeight = 0.0;
    std::size_t clusterCount = 0;
    std::size_t significantClusterCount = 0;
    double particleEntropy = 0.0;
    SensorUpdateResult sensor;
    std::string explanation;
};

struct LocalizationHistorySample {
    Pose2D estimate;
    Pose2D odometry;
    std::size_t particleCount = 0;
    double effectiveSampleSize = 0.0;
    double dominantWeight = 0.0;
    double secondWeight = 0.0;
    double positionSigma = 0.0;
    double headingSigma = 0.0;
    double recoveryProbability = 0.0;
    double observationQuality = 0.0;
};

struct LidarConfig {
    std::size_t beamCount = 91;
    double fieldOfView = 2.0 * kLocalizationPi;
    double minRange = 5.0;
    double maxRange = 800.0;
    double offsetX = 0.0;
    double offsetY = 0.0;
    double yawOffset = 0.0;
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
    bool doBeamSkip = true;
    double beamSkipDistance = 60.0;
    double beamSkipThreshold = 0.30;
    double beamSkipErrorThreshold = 0.90;

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
    std::size_t minimumSensorUpdatesForConvergence = 2;
    std::size_t minimumGlobalSensorUpdatesForConvergence = 25;
    double clusterBinSizeX = 75.0;
    double clusterBinSizeY = 75.0;
    double clusterBinSizeYaw = 0.35;
    double clusterMinimumBinWeightRatio = 0.05;
    double significantClusterWeight = 0.05;
    double dominantClusterWeight = 0.55;
    double dominantToSecondRatio = 1.50;
    double dominantSwitchMargin = 0.10;
    double minimumHeadingResultant = 0.60;
    std::size_t minimumSupportBeams = 3;
    double minimumLikelihoodContrast = 1.02;
    double recoveringProbabilityThreshold = 0.05;
    double navigationDominantWeight = 0.65;
    double navigationPositionStdDev = 30.0;
    double navigationHeadingStdDev = 0.25;
    std::size_t historyCapacity = 128;

    std::uint32_t randomSeed = 0x00C0FFEEu;
};
