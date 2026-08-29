#include "SlamOccupancyGrid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace {
bool validConfig(const SlamOccupancyGridConfig& config) {
    return std::isfinite(config.resolution) && config.resolution > 0.0
        && std::isfinite(config.originX) && std::isfinite(config.originY)
        && config.width > 0 && config.height > 0
        && std::isfinite(config.occupiedLogOddsIncrement)
        && config.occupiedLogOddsIncrement > 0.0
        && std::isfinite(config.freeLogOddsIncrement)
        && config.freeLogOddsIncrement < 0.0
        && std::isfinite(config.minimumLogOdds)
        && std::isfinite(config.maximumLogOdds)
        && config.minimumLogOdds < config.maximumLogOdds
        && std::isfinite(config.freeLogOddsThreshold)
        && std::isfinite(config.occupiedLogOddsThreshold)
        && config.freeLogOddsThreshold < config.occupiedLogOddsThreshold
        && config.freeLogOddsThreshold >= config.minimumLogOdds
        && config.occupiedLogOddsThreshold <= config.maximumLogOdds;
}
}

SlamOccupancyGrid::SlamOccupancyGrid(const SlamOccupancyGridConfig& config)
    : m_config(config) {
    if (!validConfig(config)) {
        throw std::invalid_argument("SLAM occupancy-grid configuration is invalid.");
    }
    reset();
}

void SlamOccupancyGrid::reset() {
    m_logOdds.assign(m_config.width * m_config.height, 0.0f);
    m_observed.assign(m_config.width * m_config.height, 0);
    m_revision = 0;
}

const SlamOccupancyGridConfig& SlamOccupancyGrid::getConfig() const {
    return m_config;
}

bool SlamOccupancyGrid::isInBounds(const SlamGridCoord& cell) const {
    return cell.col >= 0 && cell.row >= 0
        && static_cast<std::size_t>(cell.col) < m_config.width
        && static_cast<std::size_t>(cell.row) < m_config.height;
}

bool SlamOccupancyGrid::contains(const sf::Vector2f& point) const {
    return std::isfinite(point.x) && std::isfinite(point.y)
        && point.x >= m_config.originX && point.y >= m_config.originY
        && point.x < m_config.originX + m_config.resolution * m_config.width
        && point.y < m_config.originY + m_config.resolution * m_config.height;
}

SlamGridCoord SlamOccupancyGrid::worldToCell(const sf::Vector2f& point) const {
    return SlamGridCoord{
        static_cast<int>(std::floor((point.x - m_config.originX) / m_config.resolution)),
        static_cast<int>(std::floor((point.y - m_config.originY) / m_config.resolution))
    };
}

sf::Vector2f SlamOccupancyGrid::cellCenter(const SlamGridCoord& cell) const {
    return sf::Vector2f(
        static_cast<float>(m_config.originX + (cell.col + 0.5) * m_config.resolution),
        static_cast<float>(m_config.originY + (cell.row + 0.5) * m_config.resolution)
    );
}

std::size_t SlamOccupancyGrid::indexOf(const SlamGridCoord& cell) const {
    if (!isInBounds(cell)) {
        throw std::out_of_range("SLAM occupancy-grid cell is out of bounds.");
    }
    return static_cast<std::size_t>(cell.row) * m_config.width
        + static_cast<std::size_t>(cell.col);
}

OccupancyState SlamOccupancyGrid::getState(const SlamGridCoord& cell) const {
    const std::size_t index = indexOf(cell);
    if (m_observed[index] == 0) {
        return OccupancyState::Unknown;
    }
    if (m_logOdds[index] <= m_config.freeLogOddsThreshold) {
        return OccupancyState::Free;
    }
    if (m_logOdds[index] >= m_config.occupiedLogOddsThreshold) {
        return OccupancyState::Occupied;
    }
    return OccupancyState::Unknown;
}

double SlamOccupancyGrid::getLogOdds(const SlamGridCoord& cell) const {
    return m_logOdds[indexOf(cell)];
}

bool SlamOccupancyGrid::wasObserved(const SlamGridCoord& cell) const {
    return m_observed[indexOf(cell)] != 0;
}

bool SlamOccupancyGrid::applyEvidence(const std::vector<SlamCellEvidence>& evidence) {
    if (evidence.empty()) {
        return false;
    }
    bool changed = false;
    for (const SlamCellEvidence& item : evidence) {
        if (!isInBounds(item.cell)) {
            continue;
        }
        const std::size_t index = indexOf(item.cell);
        const double increment = item.occupied
            ? m_config.occupiedLogOddsIncrement
            : m_config.freeLogOddsIncrement;
        m_logOdds[index] = static_cast<float>(std::clamp(
            static_cast<double>(m_logOdds[index]) + increment,
            m_config.minimumLogOdds,
            m_config.maximumLogOdds
        ));
        m_observed[index] = 1;
        changed = true;
    }
    if (changed) {
        ++m_revision;
    }
    return changed;
}

double SlamOccupancyGrid::occupiedScore(const sf::Vector2f& point, int radiusCells) const {
    if (!contains(point) || radiusCells < 0) {
        return 0.0;
    }
    const SlamGridCoord center = worldToCell(point);
    double best = 0.0;
    for (int rowOffset = -radiusCells; rowOffset <= radiusCells; ++rowOffset) {
        for (int colOffset = -radiusCells; colOffset <= radiusCells; ++colOffset) {
            const SlamGridCoord cell{center.col + colOffset, center.row + rowOffset};
            if (!isInBounds(cell) || getState(cell) != OccupancyState::Occupied) {
                continue;
            }
            const int distance = std::max(std::abs(colOffset), std::abs(rowOffset));
            const double score = 1.0 / (1.0 + static_cast<double>(distance));
            best = std::max(best, score);
        }
    }
    return best;
}

SlamMapStatistics SlamOccupancyGrid::getStatistics() const {
    SlamMapStatistics statistics;
    statistics.revision = m_revision;
    for (std::size_t row = 0; row < m_config.height; ++row) {
        for (std::size_t col = 0; col < m_config.width; ++col) {
            const OccupancyState state = getState(SlamGridCoord{
                static_cast<int>(col), static_cast<int>(row)
            });
            if (state == OccupancyState::Free) {
                ++statistics.freeCells;
            } else if (state == OccupancyState::Occupied) {
                ++statistics.occupiedCells;
            } else {
                ++statistics.unknownCells;
            }
        }
    }
    return statistics;
}

std::uint64_t SlamOccupancyGrid::getRevision() const {
    return m_revision;
}
