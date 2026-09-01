#pragma once

#include <random>
#include <vector>

#include "localization/LocalizationTypes.hpp"
#include "map/MapData.hpp"
#include "localization/MapLikelihoodField.hpp"

class ParticleFilter {
public:
    explicit ParticleFilter(const AmclConfig& config = AmclConfig{});

    void clear();
    bool initializeLocal(const Pose2D& mean, const MapData& mapData, std::mt19937& randomEngine);
    bool initializeGlobal(const MapData& mapData, std::mt19937& randomEngine);
    void motionUpdate(const OdometryDelta& odometry, std::mt19937& randomEngine);
    SensorUpdateResult sensorUpdate(
        const LaserScan& scan,
        const MapLikelihoodField& field,
        bool allowBeamSkipping = true
    );
    void adaptiveResample(
        const MapData& mapData,
        double randomInjectionProbability,
        std::mt19937& randomEngine
    );

    const std::vector<Particle>& getParticles() const;
    LocalizationEstimate estimate() const;

    static bool normalizeWeights(std::vector<Particle>& particles);
    static double effectiveSampleSize(const std::vector<Particle>& particles);
    static LocalizationEstimate estimateParticles(const std::vector<Particle>& particles);
    static std::vector<ParticleCluster> clusterParticles(
        const std::vector<Particle>& particles,
        const AmclConfig& config
    );
    static double particleEntropy(const std::vector<Particle>& particles);
    static std::size_t requiredKldSamples(
        std::size_t occupiedBinCount,
        const AmclConfig& config
    );
    static std::vector<Particle> systematicResample(
        const std::vector<Particle>& particles,
        std::size_t outputCount,
        std::mt19937& randomEngine
    );

private:
    bool sampleFreePose(const MapData& mapData, Pose2D& pose, std::mt19937& randomEngine) const;
    bool isPoseFree(const Pose2D& pose, const MapData& mapData) const;
    AmclConfig m_config;
    std::vector<Particle> m_particles;
};
