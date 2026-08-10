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
    bool isFree(const sf::Vector2f& worldPoint) const;
    double distanceAt(const sf::Vector2f& worldPoint) const;
    std::uint64_t getSourceRevision() const;

private:
    sf::FloatRect m_boundary;
    double m_maxDistance = 0.0;
    std::vector<sf::FloatRect> m_obstacleBounds;
    std::size_t m_geometrySignature = 0;
    std::uint64_t m_sourceRevision = 0;
    bool m_valid = false;
};
