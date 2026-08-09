#pragma once

#include <random>
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
    const LocalizationStatistics& getStatistics() const;

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
    LocalizationStatistics m_statistics;
};
