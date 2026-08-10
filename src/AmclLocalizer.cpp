#include "AmclLocalizer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
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
        && ((config.recoveryAlphaSlow == 0.0 && config.recoveryAlphaFast == 0.0)
            || (config.recoveryAlphaSlow > 0.0
                && config.recoveryAlphaFast > config.recoveryAlphaSlow))
        && std::isfinite(config.convergencePositionStdDev)
        && config.convergencePositionStdDev >= 0.0
        && std::isfinite(config.convergenceHeadingStdDev)
        && config.convergenceHeadingStdDev >= 0.0
        && config.minimumSensorUpdatesForConvergence > 0
        && config.minimumGlobalSensorUpdatesForConvergence
            >= config.minimumSensorUpdatesForConvergence
        && config.minimumSupportBeams > 0
        && std::isfinite(config.minimumLikelihoodContrast)
        && config.minimumLikelihoodContrast >= 1.0;
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

bool hasRepeatedMapGeometry(const MapData& mapData) {
    const auto& obstacles = mapData.getObstacles();
    if (obstacles.size() < 6) {
        return false;
    }
    const std::set<GridCoord>& occupied = obstacles;
    for (const GridCoord& first : obstacles) {
        for (const GridCoord& second : obstacles) {
            const int deltaCol = second.col - first.col;
            const int deltaRow = second.row - first.row;
            if (std::abs(deltaCol) + std::abs(deltaRow) < 5) {
                continue;
            }
            std::size_t overlap = 0;
            for (const GridCoord& obstacle : obstacles) {
                if (occupied.count(GridCoord{
                        obstacle.col + deltaCol, obstacle.row + deltaRow
                    }) > 0) {
                    ++overlap;
                }
            }
            if (static_cast<double>(overlap) / static_cast<double>(obstacles.size()) >= 0.45) {
                return true;
            }
        }
    }
    return false;
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
    m_wholeFilterEstimate = LocalizationEstimate{};
    m_statistics = LocalizationStatistics{};
    m_clusters.clear();
    m_history.clear();
    m_previousDominantPose.reset();
    m_mapObstacleCount = 0;
    m_mapHasRepeatedGeometry = false;
    m_measurementSupportObserved = false;
    m_currentMeasurementUsable = false;
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
    m_mapObstacleCount = mapData.getObstacles().size();
    m_mapHasRepeatedGeometry = hasRepeatedMapGeometry(mapData);
    m_statistics.initialization = LocalizationInitialization::Local;
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
    m_mapObstacleCount = mapData.getObstacles().size();
    m_mapHasRepeatedGeometry = hasRepeatedMapGeometry(mapData);
    m_statistics.initialization = LocalizationInitialization::Global;
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
    const bool allowBeamSkipping = m_statistics.initialization
            != LocalizationInitialization::Global
        || m_statistics.state == LocalizationState::Converged;
    const SensorUpdateResult sensorResult = updatedFilter.sensorUpdate(
        scan, field, allowBeamSkipping
    );
    if (!sensorResult.updated) {
        const bool validUninformativeScan = sensorResult.selectedBeams > 0
            && sensorResult.invalidBeams + sensorResult.maxRangeBeams
                == sensorResult.selectedBeams;
        if (!validUninformativeScan) {
            return false;
        }
        m_filter = std::move(updatedFilter);
        randomEngine = updatedRandomEngine;
        m_pendingOdometry.clear();
        m_accumulatedTranslation = 0.0;
        m_accumulatedRotation = 0.0;
        m_forceSensorUpdate = false;
        m_mapObstacleCount = mapData.getObstacles().size();
        m_mapHasRepeatedGeometry = hasRepeatedMapGeometry(mapData);
        m_statistics.sensor = sensorResult;
        m_currentMeasurementUsable = false;
        m_statistics.preResampleEffectiveSampleSize = ParticleFilter::effectiveSampleSize(
            m_filter.getParticles()
        );
        m_statistics.effectiveSampleSize = m_statistics.preResampleEffectiveSampleSize;
        m_statistics.particleCount = m_filter.getParticles().size();
        refreshEstimateAndState();
        return true;
    }
    m_filter = std::move(updatedFilter);
    randomEngine = updatedRandomEngine;
    m_pendingOdometry.clear();
    m_accumulatedTranslation = 0.0;
    m_accumulatedRotation = 0.0;
    m_forceSensorUpdate = false;

    ++m_statistics.sensorUpdateCount;
    m_mapObstacleCount = mapData.getObstacles().size();
    m_mapHasRepeatedGeometry = hasRepeatedMapGeometry(mapData);
    m_statistics.sensor = sensorResult;
    m_currentMeasurementUsable = sensorResult.usedBeams >= m_config.minimumSupportBeams
        && sensorResult.observationQuality > 0.0;
    if (sensorResult.usedBeams >= m_config.minimumSupportBeams
        && sensorResult.likelihoodContrast >= m_config.minimumLikelihoodContrast) {
        m_measurementSupportObserved = true;
    }
    updateRecovery(sensorResult.observationQuality);
    m_statistics.preResampleEffectiveSampleSize = ParticleFilter::effectiveSampleSize(
        m_filter.getParticles()
    );
    m_statistics.effectiveSampleSize = m_statistics.preResampleEffectiveSampleSize;
    m_statistics.particleCount = m_filter.getParticles().size();

    const bool intervalDue = m_statistics.sensorUpdateCount % m_config.resampleInterval == 0;
    const bool essLow = m_statistics.effectiveSampleSize
        < m_config.resampleEssRatio * static_cast<double>(m_filter.getParticles().size());
    const bool recoveryActive = m_statistics.recoveryProbability > 0.0;
    const std::vector<ParticleCluster> weightedClusters = ParticleFilter::clusterParticles(
        m_filter.getParticles(), m_config
    );
    std::size_t weightedSignificantCount = 0;
    for (const ParticleCluster& cluster : weightedClusters) {
        weightedSignificantCount += cluster.weight >= m_config.significantClusterWeight ? 1 : 0;
    }
    const bool unresolvedRepeatedGeometry = m_mapHasRepeatedGeometry
        && m_statistics.initialization == LocalizationInitialization::Global
        && weightedSignificantCount > 1;
    if ((!unresolvedRepeatedGeometry || recoveryActive)
        && (intervalDue || essLow || recoveryActive)) {
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

const LocalizationEstimate& AmclLocalizer::getWholeFilterEstimate() const {
    return m_wholeFilterEstimate;
}

const LocalizationStatistics& AmclLocalizer::getStatistics() const {
    return m_statistics;
}

const std::vector<ParticleCluster>& AmclLocalizer::getClusters() const {
    return m_clusters;
}

const std::vector<LocalizationHistorySample>& AmclLocalizer::getHistory() const {
    return m_history;
}

void AmclLocalizer::appendHistory(const Pose2D& odometryPose) {
    if (!m_estimate.valid) {
        return;
    }
    LocalizationHistorySample sample;
    sample.estimate = m_estimate.pose;
    sample.odometry = odometryPose;
    sample.particleCount = m_statistics.particleCount;
    sample.effectiveSampleSize = m_statistics.preResampleEffectiveSampleSize;
    sample.dominantWeight = m_statistics.dominantClusterWeight;
    sample.secondWeight = m_statistics.secondClusterWeight;
    sample.positionSigma = std::sqrt(std::max(
        0.0, std::max(m_estimate.covariance.xx(), m_estimate.covariance.yy())
    ));
    sample.headingSigma = std::sqrt(std::max(0.0, m_estimate.covariance.yawYaw()));
    sample.recoveryProbability = m_statistics.recoveryProbability;
    sample.observationQuality = m_statistics.sensor.observationQuality;
    if (m_history.size() == m_config.historyCapacity) {
        m_history.erase(m_history.begin());
    }
    m_history.push_back(sample);
}

void AmclLocalizer::refreshEstimateAndState() {
    m_wholeFilterEstimate = m_filter.estimate();
    m_clusters = ParticleFilter::clusterParticles(m_filter.getParticles(), m_config);
    if (!m_wholeFilterEstimate.valid || m_clusters.empty()) {
        m_estimate = LocalizationEstimate{};
        m_statistics.state = LocalizationState::Uninitialized;
        m_statistics.explanation = "Particle belief is not initialized.";
        return;
    }

    std::size_t dominantIndex = 0;
    if (m_previousDominantPose.has_value()) {
        for (std::size_t index = 0; index < m_clusters.size(); ++index) {
            const double distance = std::hypot(
                m_clusters[index].pose.position.x - m_previousDominantPose->position.x,
                m_clusters[index].pose.position.y - m_previousDominantPose->position.y
            );
            const double yawDistance = std::abs(normalizeLocalizationAngle(
                m_clusters[index].pose.heading - m_previousDominantPose->heading
            ));
            if (distance <= 2.0 * std::max(m_config.clusterBinSizeX, m_config.clusterBinSizeY)
                && yawDistance <= 2.0 * m_config.clusterBinSizeYaw
                && m_clusters[index].weight + m_config.dominantSwitchMargin
                    >= m_clusters.front().weight) {
                dominantIndex = index;
                break;
            }
        }
    }
    const ParticleCluster& dominant = m_clusters[dominantIndex];
    m_previousDominantPose = dominant.pose;
    m_estimate = LocalizationEstimate{};
    m_estimate.pose = dominant.pose;
    m_estimate.covariance = dominant.covariance;
    m_estimate.valid = true;
    m_estimate.effectiveSampleSize = m_statistics.effectiveSampleSize;
    m_estimate.particleCount = m_filter.getParticles().size();

    std::size_t significantCount = 0;
    double secondWeight = 0.0;
    for (std::size_t index = 0; index < m_clusters.size(); ++index) {
        if (m_clusters[index].weight >= m_config.significantClusterWeight) {
            ++significantCount;
        }
        if (index != dominantIndex) {
            secondWeight = std::max(secondWeight, m_clusters[index].weight);
        }
    }
    m_statistics.clusterCount = m_clusters.size();
    m_statistics.significantClusterCount = significantCount;
    m_statistics.dominantClusterWeight = dominant.weight;
    m_statistics.secondClusterWeight = secondWeight;
    m_statistics.particleEntropy = ParticleFilter::particleEntropy(m_filter.getParticles());
    m_estimate.dominantWeight = dominant.weight;
    m_estimate.secondWeight = secondWeight;
    m_estimate.significantClusterCount = significantCount;

    const double dominanceRatio = secondWeight > 1e-12
        ? dominant.weight / secondWeight
        : std::numeric_limits<double>::infinity();
    const bool unresolvedRepeatedGeometry = m_mapHasRepeatedGeometry
        && m_statistics.initialization == LocalizationInitialization::Global
        && significantCount > 1
        && (dominant.weight < m_config.dominantClusterWeight
            || dominanceRatio < m_config.dominantToSecondRatio);
    if (m_mapObstacleCount == 0) {
        m_statistics.support = LocalizationSupport::Insufficient;
    } else if (unresolvedRepeatedGeometry || m_statistics.sensorUpdateCount == 0
        || significantCount > 1 || !m_measurementSupportObserved
        || !m_currentMeasurementUsable) {
        m_statistics.support = LocalizationSupport::Weak;
    } else {
        m_statistics.support = LocalizationSupport::Good;
    }

    const double sigmaX = std::sqrt(std::max(0.0, m_estimate.covariance.xx()));
    const double sigmaY = std::sqrt(std::max(0.0, m_estimate.covariance.yy()));
    const double sigmaYaw = std::sqrt(std::max(0.0, m_estimate.covariance.yawYaw()));
    const bool ambiguous = unresolvedRepeatedGeometry
        || (significantCount > 1
            && (dominant.weight < m_config.dominantClusterWeight
                || dominanceRatio < m_config.dominantToSecondRatio));
    const std::size_t requiredUpdates = m_statistics.initialization
            == LocalizationInitialization::Global
        ? m_config.minimumGlobalSensorUpdatesForConvergence
        : m_config.minimumSensorUpdatesForConvergence;
    const bool converged = m_statistics.sensorUpdateCount >= requiredUpdates
        && m_statistics.support == LocalizationSupport::Good
        && !ambiguous
        && dominant.weight >= m_config.dominantClusterWeight
        && dominant.headingResultant >= m_config.minimumHeadingResultant
        && sigmaX <= m_config.convergencePositionStdDev
        && sigmaY <= m_config.convergencePositionStdDev
        && sigmaYaw <= m_config.convergenceHeadingStdDev;
    if (m_statistics.recoveryProbability > m_config.recoveringProbabilityThreshold) {
        m_statistics.state = LocalizationState::Recovering;
        m_statistics.explanation = "Measurement mismatch is driving recovery injection.";
    } else if (ambiguous) {
        m_statistics.state = LocalizationState::Ambiguous;
        m_statistics.explanation = "Multiple particle hypotheses have comparable support.";
    } else if (converged) {
        m_statistics.state = LocalizationState::Converged;
        m_statistics.explanation = "A supported dominant hypothesis is concentrated.";
    } else {
        m_statistics.state = LocalizationState::Tracking;
        m_statistics.explanation = m_statistics.support == LocalizationSupport::Insufficient
            ? "Map geometry is insufficient for a trustworthy unique pose."
            : "More measurement evidence is required.";
    }
    m_estimate.converged = m_statistics.state == LocalizationState::Converged;
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
