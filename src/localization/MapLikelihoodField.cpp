#include "localization/MapLikelihoodField.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <stdexcept>

namespace {
void combineHash(std::size_t& seed, std::size_t value) {
    seed ^= value + 0x9e3779b9u + (seed << 6) + (seed >> 2);
}

std::size_t geometrySignature(const MapData& mapData) {
    std::size_t signature = 0;
    const sf::FloatRect& boundary = mapData.getWorldBoundary();
    const std::hash<float> floatHash;
    const std::hash<int> intHash;
    combineHash(signature, floatHash(mapData.getGridResolution()));
    combineHash(signature, floatHash(boundary.position.x));
    combineHash(signature, floatHash(boundary.position.y));
    combineHash(signature, floatHash(boundary.size.x));
    combineHash(signature, floatHash(boundary.size.y));
    for (const GridCoord& obstacle : mapData.getObstacles()) {
        combineHash(signature, intHash(obstacle.col));
        combineHash(signature, intHash(obstacle.row));
    }
    return signature;
}

double distanceToRectangle(const sf::Vector2f& point, const sf::FloatRect& rectangle) {
    const double left = rectangle.position.x;
    const double top = rectangle.position.y;
    const double right = rectangle.position.x + rectangle.size.x;
    const double bottom = rectangle.position.y + rectangle.size.y;
    const double dx = std::max({left - point.x, 0.0, point.x - right});
    const double dy = std::max({top - point.y, 0.0, point.y - bottom});
    return std::hypot(dx, dy);
}

double distanceToBoundarySurface(const sf::Vector2f& point, const sf::FloatRect& boundary) {
    const double left = boundary.position.x;
    const double top = boundary.position.y;
    const double right = boundary.position.x + boundary.size.x;
    const double bottom = boundary.position.y + boundary.size.y;
    const bool inside = point.x >= left && point.x <= right
        && point.y >= top && point.y <= bottom;
    if (inside) {
        return std::min({
            static_cast<double>(point.x) - left,
            right - static_cast<double>(point.x),
            static_cast<double>(point.y) - top,
            bottom - static_cast<double>(point.y)
        });
    }
    return distanceToRectangle(point, boundary);
}
}

void MapLikelihoodField::rebuild(const MapData& mapData, double maxDistance) {
    const sf::FloatRect boundary = mapData.getWorldBoundary();
    const double resolution = mapData.getGridResolution();
    if (!std::isfinite(maxDistance) || maxDistance <= 0.0
        || !std::isfinite(resolution) || resolution <= 0.0
        || !std::isfinite(boundary.position.x) || !std::isfinite(boundary.position.y)
        || !std::isfinite(boundary.size.x) || !std::isfinite(boundary.size.y)
        || boundary.size.x <= 0.0f || boundary.size.y <= 0.0f) {
        throw std::invalid_argument("Likelihood-field configuration is invalid.");
    }

    std::vector<sf::FloatRect> obstacleBounds;
    obstacleBounds.reserve(mapData.getObstacles().size());
    const double gridSize = mapData.getGridResolution();
    for (const GridCoord& obstacle : mapData.getObstacles()) {
        obstacleBounds.emplace_back(
            mapData.getMapper().gridToWorldTopLeft(obstacle),
            sf::Vector2f(static_cast<float>(gridSize), static_cast<float>(gridSize))
        );
    }
    m_boundary = boundary;
    m_maxDistance = maxDistance;
    m_obstacleBounds = std::move(obstacleBounds);
    m_geometrySignature = geometrySignature(mapData);
    m_sourceRevision = mapData.getGeometryRevision();
    m_valid = true;
}

void MapLikelihoodField::rebuildIfNeeded(const MapData& mapData, double maxDistance) {
    if (!m_valid || m_sourceRevision != mapData.getGeometryRevision()
        || m_geometrySignature != geometrySignature(mapData)
        || m_maxDistance != maxDistance) {
        rebuild(mapData, maxDistance);
    }
}

void MapLikelihoodField::clear() {
    m_obstacleBounds.clear();
    m_geometrySignature = 0;
    m_valid = false;
}

bool MapLikelihoodField::isValid() const {
    return m_valid;
}

bool MapLikelihoodField::isFree(const sf::Vector2f& worldPoint) const {
    if (!m_valid || !std::isfinite(worldPoint.x) || !std::isfinite(worldPoint.y)
        || !m_boundary.contains(worldPoint)) {
        return false;
    }
    return std::none_of(
        m_obstacleBounds.begin(), m_obstacleBounds.end(),
        [&](const sf::FloatRect& obstacle) { return obstacle.contains(worldPoint); }
    );
}

double MapLikelihoodField::distanceAt(const sf::Vector2f& worldPoint) const {
    if (!m_valid || !std::isfinite(worldPoint.x) || !std::isfinite(worldPoint.y)) {
        return m_maxDistance;
    }

    const double boundaryDistance = distanceToBoundarySurface(worldPoint, m_boundary);
    const double boundaryRight = m_boundary.position.x + m_boundary.size.x;
    const double boundaryBottom = m_boundary.position.y + m_boundary.size.y;
    const bool insideOrOnBoundary = worldPoint.x >= m_boundary.position.x
        && worldPoint.x <= boundaryRight
        && worldPoint.y >= m_boundary.position.y
        && worldPoint.y <= boundaryBottom;
    if (!insideOrOnBoundary) {
        return std::clamp(boundaryDistance, 0.0, m_maxDistance);
    }

    // Obstacles are static grid-cell AABBs. Caching those surfaces makes the
    // continuous query exact, including faces and diagonal corners, while the
    // field remains map-derived and is rebuilt only when geometry changes.
    double obstacleDistance = m_maxDistance;
    for (const sf::FloatRect& bounds : m_obstacleBounds) {
        if (worldPoint.x < bounds.position.x - obstacleDistance
            || worldPoint.x > bounds.position.x + bounds.size.x + obstacleDistance
            || worldPoint.y < bounds.position.y - obstacleDistance
            || worldPoint.y > bounds.position.y + bounds.size.y + obstacleDistance) {
            continue;
        }
        obstacleDistance = std::min(obstacleDistance, distanceToRectangle(worldPoint, bounds));
    }

    return std::clamp(
        std::min(obstacleDistance, boundaryDistance),
        0.0,
        m_maxDistance
    );
}

std::uint64_t MapLikelihoodField::getSourceRevision() const {
    return m_sourceRevision;
}
