#pragma once

#include <random>

#include "LocalizationTypes.hpp"
#include "MapData.hpp"

class LidarSimulator {
public:
    explicit LidarSimulator(const LidarConfig& config = LidarConfig{});

    const LidarConfig& getConfig() const;
    LaserScan simulate(
        const Pose2D& groundTruthPose,
        const MapData& mapData,
        std::mt19937& randomEngine
    ) const;

private:
    LidarConfig m_config;
};
