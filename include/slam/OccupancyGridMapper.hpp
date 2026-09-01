#pragma once

#include <vector>

#include "slam/SlamOccupancyGrid.hpp"

class OccupancyGridMapper {
public:
    OccupancyIntegrationResult integrate(
        SlamOccupancyGrid& grid,
        const LaserScan& scan,
        const Pose2D& basePose
    ) const;

    static bool isScanMetadataValid(const LaserScan& scan);
    static std::vector<SlamGridCoord> traceRay(
        const SlamOccupancyGrid& grid,
        const sf::Vector2f& start,
        const sf::Vector2f& end
    );
};
