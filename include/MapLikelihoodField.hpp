#pragma once

#include <cstdint>
#include <vector>

#include "MapData.hpp"

class MapLikelihoodField {
public:
    void rebuild(const MapData& mapData, double maxDistance);
    void rebuildIfNeeded(const MapData& mapData, double maxDistance);
    void clear();

    bool isValid() const;
    double distanceAt(const sf::Vector2f& worldPoint) const;
    std::uint64_t getSourceRevision() const;

private:
    std::size_t sampleIndex(int col, int row) const;

    sf::FloatRect m_boundary;
    double m_resolution = 0.0;
    double m_maxDistance = 0.0;
    int m_minCol = 0;
    int m_minRow = 0;
    int m_maxCol = -1;
    int m_maxRow = -1;
    std::vector<double> m_obstacleDistances;
    std::size_t m_geometrySignature = 0;
    std::uint64_t m_sourceRevision = 0;
    bool m_valid = false;
};
