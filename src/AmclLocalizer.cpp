#include "AmclLocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {
void validateCoordinatorConfig(const AmclConfig& config) {
    const bool valid = std::isfinite(config.updateMinTranslation)
        && std::isfinite(config.updateMinRotation)
        && config.updateMinTranslation >= 0.0
        && config.updateMinRotation >= 0.0
        && config.resampleInterval > 0
        && std::isfinite(config.resampleEssRatio)
        && config.resampleEssRatio >= 0.0
        && config.resampleEssRatio <= 1.0
        && std::isfinite(config.recoveryAlphaSlow)
        && std::isfinite(config.recoveryAlphaFast)
        && config.recoveryAlphaSlow >= 0.0
        && config.recoveryAlphaSlow <= 1.0
        && config.recoveryAlphaFast >= 0.0
        && config.recoveryAlphaFast <= 1.0
        && std::isfinite(config.convergencePositionStdDev)
        && config.convergencePositionStdDev >= 0.0
        && std::isfinite(config.convergenceHeadingStdDev)
        && config.convergenceHeadingStdDev >= 0.0;
    if (!valid) {
        throw std::invalid_argument("AMCL coordinator configuration is invalid.");
    }
}

OdometryDelta composeOdometry(const std::vector<OdometryDelta>& increments) {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    for (const OdometryDelta& increment : increments) {
        const double movementHeading = heading + increment.rotation1;
        x += increment.translation * std::cos(movementHeading);
        y += increment.translation * std::sin(movementHeading);
        heading = normalizeLocalizationAngle(
            heading + increment.rotation1 + increment.rotation2
        );
    }

    const double distance = std::hypot(x, y);
    double rotation1 = 0.0;
    double translation = 0.0;
    if (distance > 1e-12) {
        const bool reverse = x < 0.0;
        rotation1 = normalizeLocalizationAngle(std::atan2(
            reverse ? -y : y,
            reverse ? -x : x
        ));
        translation = reverse ? -distance : distance;
    }
    return OdometryDelta{
        rotation1,
        translation,
        normalizeLocalizationAngle(heading - rotation1),
        true
    };
}
}

AmclLocalizer::AmclLocalizer(const AmclConfig& config)
    : m_config(config), m_filter(config) {
    validateCoordinatorConfig(m_config);
}

void AmclLocalizer::reset() {
    m_filter.clear();
    m_pendingOdometry.clear();
    m_accumulatedTranslation = 0.0;
    m_accumulatedRotation = 0.0;
    m_forceSensorUpdate = false;
    m_estimate = LocalizationEstimate{};
    m_statistics = LocalizationStatistics{};
}

bool AmclLocalizer::initializeLocal(
    const Pose2D& mean,
    const MapData& mapData,
    std::mt19937& randomEngine
) {
    reset();
    if (!m_filter.initializeLocal(mean, mapData, randomEngine)) {
        return false;
    }
    m_forceSensorUpdate = true;
    m_statistics.state = LocalizationState::Tracking;
    m_statistics.particleCount = m_filter.getParticles().size();
    m_statistics.effectiveSampleSize = ParticleFilter::effectiveSampleSize(
        m_filter.getParticles()
    );
    refreshEstimateAndState();
    return true;
}

bool AmclLocalizer::initializeGlobal(
    const MapData& mapData,
    std::mt19937& randomEngine
) {
    reset();
    if (!m_filter.initializeGlobal(mapData, randomEngine)) {
        return false;
    }
    m_forceSensorUpdate = true;
    m_statistics.state = LocalizationState::Tracking;
    m_statistics.particleCount = m_filter.getParticles().size();
    m_statistics.effectiveSampleSize = ParticleFilter::effectiveSampleSize(
        m_filter.getParticles()
    );
    refreshEstimateAndState();
    return true;
}

bool AmclLocalizer::accumulateOdometry(const OdometryDelta& odometry) {
    if (!odometry.valid || !std::isfinite(odometry.rotation1)
        || !std::isfinite(odometry.translation)
        || !std::isfinite(odometry.rotation2)) {
        return false;
    }

    const bool hasMotion = std::abs(odometry.rotation1) > 1e-12
        || std::abs(odometry.translation) > 1e-12
        || std::abs(odometry.rotation2) > 1e-12;
    if (hasMotion) {
        m_pendingOdometry.push_back(odometry);
        m_accumulatedTranslation += std::abs(odometry.translation);
        m_accumulatedRotation += std::abs(odometry.rotation1)
            + std::abs(odometry.rotation2);
    }
    return needsSensorUpdate();
}

bool AmclLocalizer::needsSensorUpdate() const {
    if (m_filter.getParticles().empty()) {
        return false;
    }
    if (m_forceSensorUpdate) {
        return true;
    }
    if (m_pendingOdometry.empty()) {
        return false;
    }
    return m_accumulatedTranslation >= m_config.updateMinTranslation
        || m_accumulatedRotation >= m_config.updateMinRotation;
}

bool AmclLocalizer::updateWithScan(
    const LaserScan& scan,
    const MapLikelihoodField& field,
    const MapData& mapData,
    std::mt19937& randomEngine
) {
    if (!needsSensorUpdate() || !field.isValid()) {
        return false;
    }

    ParticleFilter updatedFilter = m_filter;
    std::mt19937 updatedRandomEngine = randomEngine;
    if (!m_pendingOdometry.empty()) {
        updatedFilter.motionUpdate(composeOdometry(m_pendingOdometry), updatedRandomEngine);
    }
    const SensorUpdateResult sensorResult = updatedFilter.sensorUpdate(scan, field);
    if (!sensorResult.updated) {
        return false;
    }
    m_filter = std::move(updatedFilter);
    randomEngine = updatedRandomEngine;
    m_pendingOdometry.clear();
    m_accumulatedTranslation = 0.0;
    m_accumulatedRotation = 0.0;
    m_forceSensorUpdate = false;

    ++m_statistics.sensorUpdateCount;
    updateRecovery(sensorResult.observationQuality);
    m_statistics.effectiveSampleSize = ParticleFilter::effectiveSampleSize(
        m_filter.getParticles()
    );
    m_statistics.particleCount = m_filter.getParticles().size();

    const bool intervalDue = m_statistics.sensorUpdateCount % m_config.resampleInterval == 0;
    const bool essLow = m_statistics.effectiveSampleSize
        < m_config.resampleEssRatio * static_cast<double>(m_filter.getParticles().size());
    const bool recoveryActive = m_statistics.recoveryProbability > 0.0;
    if (intervalDue || essLow || recoveryActive) {
        m_filter.adaptiveResample(
            mapData,
            m_statistics.recoveryProbability,
            randomEngine
        );
        m_statistics.particleCount = m_filter.getParticles().size();
        m_statistics.effectiveSampleSize = ParticleFilter::effectiveSampleSize(
            m_filter.getParticles()
        );
    }

    refreshEstimateAndState();
    return true;
}

void AmclLocalizer::forceSensorUpdate() {
    if (!m_filter.getParticles().empty()) {
        m_forceSensorUpdate = true;
    }
}

const std::vector<Particle>& AmclLocalizer::getParticles() const {
    return m_filter.getParticles();
}

const LocalizationEstimate& AmclLocalizer::getEstimate() const {
    return m_estimate;
}

const LocalizationStatistics& AmclLocalizer::getStatistics() const {
    return m_statistics;
}

void AmclLocalizer::refreshEstimateAndState() {
    m_estimate = m_filter.estimate();
    if (!m_estimate.valid) {
        m_statistics.state = LocalizationState::Uninitialized;
        return;
    }
    m_estimate.effectiveSampleSize = m_statistics.effectiveSampleSize;
    m_estimate.particleCount = m_filter.getParticles().size();

    const double sigmaX = std::sqrt(std::max(0.0, m_estimate.covariance.xx()));
    const double sigmaY = std::sqrt(std::max(0.0, m_estimate.covariance.yy()));
    const double sigmaYaw = std::sqrt(std::max(0.0, m_estimate.covariance.yawYaw()));
    const bool converged = sigmaX <= m_config.convergencePositionStdDev
        && sigmaY <= m_config.convergencePositionStdDev
        && sigmaYaw <= m_config.convergenceHeadingStdDev;
    m_estimate.converged = converged;

    constexpr double kRecoveringProbabilityThreshold = 0.05;
    if (m_statistics.recoveryProbability > kRecoveringProbabilityThreshold) {
        m_statistics.state = LocalizationState::Recovering;
    } else if (converged) {
        m_statistics.state = LocalizationState::Converged;
    } else {
        m_statistics.state = LocalizationState::Tracking;
    }
}

void AmclLocalizer::updateRecovery(double observationQuality) {
    if (m_config.recoveryAlphaSlow == 0.0 || m_config.recoveryAlphaFast == 0.0) {
        m_statistics.slowWeightAverage = 0.0;
        m_statistics.fastWeightAverage = 0.0;
        m_statistics.recoveryProbability = 0.0;
        return;
    }

    const double quality = std::isfinite(observationQuality)
        ? std::max(0.0, observationQuality)
        : 0.0;
    if (m_statistics.slowWeightAverage <= 0.0) {
        m_statistics.slowWeightAverage = quality;
        m_statistics.fastWeightAverage = quality;
    } else {
        m_statistics.slowWeightAverage += m_config.recoveryAlphaSlow
            * (quality - m_statistics.slowWeightAverage);
        m_statistics.fastWeightAverage += m_config.recoveryAlphaFast
            * (quality - m_statistics.fastWeightAverage);
    }

    if (m_statistics.slowWeightAverage <= 1e-15) {
        m_statistics.recoveryProbability = 0.0;
        return;
    }
    m_statistics.recoveryProbability = std::clamp(
        1.0 - m_statistics.fastWeightAverage / m_statistics.slowWeightAverage,
        0.0,
        1.0
    );
}
