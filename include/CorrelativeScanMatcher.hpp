#pragma once

#include "SlamOccupancyGrid.hpp"

class CorrelativeScanMatcher {
public:
    explicit CorrelativeScanMatcher(
        const CorrelativeScanMatcherConfig& config = CorrelativeScanMatcherConfig{}
    );

    const CorrelativeScanMatcherConfig& getConfig() const;
    ScanMatchResult match(
        const SlamOccupancyGrid& grid,
        const LaserScan& scan,
        const Pose2D& predictedPose
    ) const;

private:
    CorrelativeScanMatcherConfig m_config;
};
