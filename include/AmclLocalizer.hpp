#pragma once

#include <random>
#include <optional>
#include <vector>

#include "ParticleFilter.hpp"

class AmclLocalizer {
public:
    explicit AmclLocalizer(const AmclConfig& config = AmclConfig{});

    void reset();
    bool initializeLocal(const Pose2D& mean, const MapData& mapData, std::mt19937& randomEngine);
    bool initializeGlobal(const MapData& mapData, std::mt19937& randomEngine);

    bool accumulateOdometry(const OdometryDelta& odometry);
    bool needsSensorUpdate() const;
    bool updateWithScan(
        const LaserScan& scan,
        const MapLikelihoodField& field,
        const MapData& mapData,
        std::mt19937& randomEngine
    );
    void forceSensorUpdate();

    const std::vector<Particle>& getParticles() const;
    const LocalizationEstimate& getEstimate() const;
    const LocalizationEstimate& getWholeFilterEstimate() const;
    const LocalizationStatistics& getStatistics() const;
    const std::vector<ParticleCluster>& getClusters() const;
    const std::vector<LocalizationHistorySample>& getHistory() const;
    void appendHistory(const Pose2D& odometryPose);

private:
    void refreshEstimateAndState();
    void updateRecovery(double observationQuality);

    AmclConfig m_config;
    ParticleFilter m_filter;
    std::vector<OdometryDelta> m_pendingOdometry;
    double m_accumulatedTranslation = 0.0;
    double m_accumulatedRotation = 0.0;
    bool m_forceSensorUpdate = false;
    LocalizationEstimate m_estimate;
    LocalizationEstimate m_wholeFilterEstimate;
    LocalizationStatistics m_statistics;
    std::vector<ParticleCluster> m_clusters;
    std::vector<LocalizationHistorySample> m_history;
    std::optional<Pose2D> m_previousDominantPose;
    std::size_t m_mapObstacleCount = 0;
    bool m_mapHasRepeatedGeometry = false;
    bool m_measurementSupportObserved = false;
    bool m_currentMeasurementUsable = false;
};
