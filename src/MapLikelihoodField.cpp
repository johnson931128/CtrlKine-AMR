#include "MapLikelihoodField.hpp"

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

    const int minCol = static_cast<int>(std::floor(boundary.position.x / resolution));
    const int minRow = static_cast<int>(std::floor(boundary.position.y / resolution));
    const int maxCol = static_cast<int>(std::ceil(
        (boundary.position.x + boundary.size.x) / resolution
    ));
    const int maxRow = static_cast<int>(std::ceil(
        (boundary.position.y + boundary.size.y) / resolution
    ));
    const std::size_t columnCount = static_cast<std::size_t>(maxCol - minCol + 1);
    const std::size_t rowCount = static_cast<std::size_t>(maxRow - minRow + 1);
    if (columnCount < 2 || rowCount < 2
        || columnCount > 100000
        || rowCount > 100000
        || columnCount > (10000000 / rowCount)) {
        throw std::invalid_argument("Likelihood-field sample dimensions are invalid.");
    }

    std::vector<double> samples(columnCount * rowCount, maxDistance);
    const double gridSize = mapData.getGridResolution();
    for (int row = minRow; row <= maxRow; ++row) {
        for (int col = minCol; col <= maxCol; ++col) {
            const sf::Vector2f samplePoint(
                static_cast<float>(col * resolution),
                static_cast<float>(row * resolution)
            );
            double nearest = maxDistance;
            for (const GridCoord& obstacle : mapData.getObstacles()) {
                const sf::Vector2f topLeft = mapData.getMapper().gridToWorldTopLeft(obstacle);
                const sf::FloatRect bounds(
                    topLeft,
                    sf::Vector2f(static_cast<float>(gridSize), static_cast<float>(gridSize))
                );
                nearest = std::min(nearest, distanceToRectangle(samplePoint, bounds));
            }
            const std::size_t index = static_cast<std::size_t>(row - minRow) * columnCount
                + static_cast<std::size_t>(col - minCol);
            samples[index] = std::min(nearest, maxDistance);
        }
    }

    m_boundary = boundary;
    m_resolution = resolution;
    m_maxDistance = maxDistance;
    m_minCol = minCol;
    m_minRow = minRow;
    m_maxCol = maxCol;
    m_maxRow = maxRow;
    m_obstacleDistances = std::move(samples);
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
    m_obstacleDistances.clear();
    m_geometrySignature = 0;
    m_valid = false;
}

bool MapLikelihoodField::isValid() const {
    return m_valid;
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

    const double gridX = worldPoint.x / m_resolution;
    const double gridY = worldPoint.y / m_resolution;
    int leftCol = static_cast<int>(std::floor(gridX));
    int topRow = static_cast<int>(std::floor(gridY));
    double tx = gridX - leftCol;
    double ty = gridY - topRow;

    leftCol = std::clamp(leftCol, m_minCol, m_maxCol - 1);
    topRow = std::clamp(topRow, m_minRow, m_maxRow - 1);
    tx = std::clamp(gridX - leftCol, 0.0, 1.0);
    ty = std::clamp(gridY - topRow, 0.0, 1.0);

    const double topLeft = m_obstacleDistances[sampleIndex(leftCol, topRow)];
    const double topRight = m_obstacleDistances[sampleIndex(leftCol + 1, topRow)];
    const double bottomLeft = m_obstacleDistances[sampleIndex(leftCol, topRow + 1)];
    const double bottomRight = m_obstacleDistances[sampleIndex(leftCol + 1, topRow + 1)];
    const double top = topLeft + (topRight - topLeft) * tx;
    const double bottom = bottomLeft + (bottomRight - bottomLeft) * tx;
    const double obstacleDistance = top + (bottom - top) * ty;

    return std::clamp(
        std::min(obstacleDistance, boundaryDistance),
        0.0,
        m_maxDistance
    );
}

std::uint64_t MapLikelihoodField::getSourceRevision() const {
    return m_sourceRevision;
}

std::size_t MapLikelihoodField::sampleIndex(int col, int row) const {
    const std::size_t columnCount = static_cast<std::size_t>(m_maxCol - m_minCol + 1);
    return static_cast<std::size_t>(row - m_minRow) * columnCount
        + static_cast<std::size_t>(col - m_minCol);
}
