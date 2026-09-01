#pragma once

#include <cstdint>

#include "slam/SlamFrontend.hpp"

struct SlamViewOptions {
    bool occupancyMap = true;
    bool pose = true;
    bool predictedPose = true;
};

class SlamVisualization {
public:
    const SlamViewOptions& getOptions() const;
    SlamViewOptions& getOptions();

    void rebuildMapIfNeeded(
        const SlamOccupancyGrid& map,
        const Pose2D& displayOrigin
    );
    void clear();
    void draw(
        sf::RenderWindow& window,
        const SlamUpdateResult& update,
        const Pose2D& displayOrigin
    ) const;

    static Pose2D toDisplayPose(const Pose2D& localPose, const Pose2D& displayOrigin);

private:
    SlamViewOptions m_options;
    sf::VertexArray m_freeCells;
    sf::VertexArray m_occupiedCells;
    std::uint64_t m_cachedRevision = static_cast<std::uint64_t>(-1);
};
