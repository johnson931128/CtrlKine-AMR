#pragma once

#include <cstdint>
#include <vector>

#include "SlamTypes.hpp"

struct SlamCellEvidence {
    SlamGridCoord cell;
    bool occupied = false;
};

class SlamOccupancyGrid {
public:
    explicit SlamOccupancyGrid(
        const SlamOccupancyGridConfig& config = SlamOccupancyGridConfig{}
    );

    void reset();
    const SlamOccupancyGridConfig& getConfig() const;
    bool isInBounds(const SlamGridCoord& cell) const;
    bool contains(const sf::Vector2f& point) const;
    SlamGridCoord worldToCell(const sf::Vector2f& point) const;
    sf::Vector2f cellCenter(const SlamGridCoord& cell) const;
    OccupancyState getState(const SlamGridCoord& cell) const;
    double getLogOdds(const SlamGridCoord& cell) const;
    bool wasObserved(const SlamGridCoord& cell) const;
    bool applyEvidence(const std::vector<SlamCellEvidence>& evidence);
    double occupiedScore(const sf::Vector2f& point, int radiusCells) const;
    SlamMapStatistics getStatistics() const;
    std::uint64_t getRevision() const;

private:
    std::size_t indexOf(const SlamGridCoord& cell) const;

    SlamOccupancyGridConfig m_config;
    std::vector<float> m_logOdds;
    std::vector<std::uint8_t> m_observed;
    std::uint64_t m_revision = 0;
};
