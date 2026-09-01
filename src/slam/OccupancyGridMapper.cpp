#include "slam/OccupancyGridMapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>

#include "sensors/LaserScanGeometry.hpp"

namespace {
bool finitePose(const Pose2D& pose) {
    return std::isfinite(pose.position.x) && std::isfinite(pose.position.y)
        && std::isfinite(pose.heading);
}

bool clipSegment(
    const SlamOccupancyGrid& grid,
    const sf::Vector2f& start,
    const sf::Vector2f& end,
    sf::Vector2f& clippedStart,
    sf::Vector2f& clippedEnd
) {
    const SlamOccupancyGridConfig& config = grid.getConfig();
    // Keep the clipped upper endpoint representably inside the half-open grid
    // after conversion to SFML's float coordinates.
    const double epsilon = config.resolution * 1e-5;
    const double minimumX = config.originX;
    const double minimumY = config.originY;
    const double maximumX = config.originX + config.resolution * config.width - epsilon;
    const double maximumY = config.originY + config.resolution * config.height - epsilon;
    const double dx = end.x - start.x;
    const double dy = end.y - start.y;
    double first = 0.0;
    double last = 1.0;

    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {
        start.x - minimumX, maximumX - start.x,
        start.y - minimumY, maximumY - start.y
    };
    for (int side = 0; side < 4; ++side) {
        if (std::abs(p[side]) <= 1e-12) {
            if (q[side] < 0.0) {
                return false;
            }
            continue;
        }
        const double ratio = q[side] / p[side];
        if (p[side] < 0.0) {
            first = std::max(first, ratio);
        } else {
            last = std::min(last, ratio);
        }
        if (first > last) {
            return false;
        }
    }

    clippedStart = sf::Vector2f(
        static_cast<float>(start.x + first * dx),
        static_cast<float>(start.y + first * dy)
    );
    clippedEnd = sf::Vector2f(
        static_cast<float>(start.x + last * dx),
        static_cast<float>(start.y + last * dy)
    );
    return grid.contains(clippedStart) && grid.contains(clippedEnd);
}
}

bool OccupancyGridMapper::isScanMetadataValid(const LaserScan& scan) {
    return !scan.ranges.empty()
        && std::isfinite(scan.angleMin)
        && std::isfinite(scan.angleIncrement)
        && std::isfinite(scan.minRange)
        && std::isfinite(scan.maxRange)
        && scan.minRange >= 0.0f && scan.minRange < scan.maxRange
        && std::isfinite(scan.sensorOffsetX)
        && std::isfinite(scan.sensorOffsetY)
        && std::isfinite(scan.sensorYawOffset);
}

std::vector<SlamGridCoord> OccupancyGridMapper::traceRay(
    const SlamOccupancyGrid& grid,
    const sf::Vector2f& start,
    const sf::Vector2f& end
) {
    std::vector<SlamGridCoord> cells;
    if (!std::isfinite(start.x) || !std::isfinite(start.y)
        || !std::isfinite(end.x) || !std::isfinite(end.y)) {
        return cells;
    }

    sf::Vector2f clippedStart;
    sf::Vector2f clippedEnd;
    if (!clipSegment(grid, start, end, clippedStart, clippedEnd)) {
        return cells;
    }

    SlamGridCoord current = grid.worldToCell(clippedStart);
    const SlamGridCoord target = grid.worldToCell(clippedEnd);
    cells.push_back(current);
    if (current == target) {
        return cells;
    }

    const double dx = clippedEnd.x - clippedStart.x;
    const double dy = clippedEnd.y - clippedStart.y;
    const int stepX = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    const int stepY = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
    const double infinity = std::numeric_limits<double>::infinity();
    const double deltaX = stepX == 0 ? infinity : grid.getConfig().resolution / std::abs(dx);
    const double deltaY = stepY == 0 ? infinity : grid.getConfig().resolution / std::abs(dy);

    const double nextBoundaryX = grid.getConfig().originX
        + (current.col + (stepX > 0 ? 1 : 0)) * grid.getConfig().resolution;
    const double nextBoundaryY = grid.getConfig().originY
        + (current.row + (stepY > 0 ? 1 : 0)) * grid.getConfig().resolution;
    double maximumX = stepX == 0 ? infinity : (nextBoundaryX - clippedStart.x) / dx;
    double maximumY = stepY == 0 ? infinity : (nextBoundaryY - clippedStart.y) / dy;

    const std::size_t limit = grid.getConfig().width * grid.getConfig().height + 1;
    while (!(current == target) && cells.size() < limit) {
        if (maximumX < maximumY) {
            current.col += stepX;
            maximumX += deltaX;
        } else if (maximumY < maximumX) {
            current.row += stepY;
            maximumY += deltaY;
        } else {
            current.col += stepX;
            current.row += stepY;
            maximumX += deltaX;
            maximumY += deltaY;
        }
        if (!grid.isInBounds(current)) {
            break;
        }
        cells.push_back(current);
    }
    return cells;
}

OccupancyIntegrationResult OccupancyGridMapper::integrate(
    SlamOccupancyGrid& grid,
    const LaserScan& scan,
    const Pose2D& basePose
) const {
    OccupancyIntegrationResult result;
    result.totalBeams = scan.ranges.size();
    if (!isScanMetadataValid(scan) || !finitePose(basePose)) {
        result.invalidBeams = scan.ranges.size();
        return result;
    }

    const double cosine = std::cos(basePose.heading);
    const double sine = std::sin(basePose.heading);
    const sf::Vector2f sensorOrigin(
        basePose.position.x + static_cast<float>(cosine * scan.sensorOffsetX - sine * scan.sensorOffsetY),
        basePose.position.y + static_cast<float>(sine * scan.sensorOffsetX + cosine * scan.sensorOffsetY)
    );

    std::map<std::size_t, SlamCellEvidence> evidenceByIndex;
    for (std::size_t beam = 0; beam < scan.ranges.size(); ++beam) {
        const double measuredRange = scan.ranges[beam];
        if (!std::isfinite(measuredRange) || measuredRange < scan.minRange) {
            ++result.invalidBeams;
            continue;
        }
        const bool hit = isLaserReturnHit(scan, measuredRange);
        const double range = hit ? measuredRange : scan.maxRange;
        const double angle = basePose.heading + scan.sensorYawOffset
            + laserScanBeamAngle(scan, beam);
        const double directionX = std::cos(angle);
        const double directionY = std::sin(angle);
        sf::Vector2f endpoint(
            sensorOrigin.x + static_cast<float>(range * directionX),
            sensorOrigin.y + static_cast<float>(range * directionY)
        );
        sf::Vector2f traversalEnd = endpoint;
        SlamGridCoord occupiedCell;
        bool occupiedInBounds = false;
        if (hit) {
            const double epsilon = grid.getConfig().resolution * 1e-4;
            const sf::Vector2f beyond(
                endpoint.x + static_cast<float>(epsilon * directionX),
                endpoint.y + static_cast<float>(epsilon * directionY)
            );
            if (grid.contains(beyond)) {
                occupiedCell = grid.worldToCell(beyond);
                occupiedInBounds = true;
                traversalEnd = beyond;
            }
            ++result.hitBeams;
        } else {
            ++result.maxRangeBeams;
        }

        const std::vector<SlamGridCoord> cells = traceRay(grid, sensorOrigin, traversalEnd);
        for (const SlamGridCoord& cell : cells) {
            const bool occupied = occupiedInBounds && cell == occupiedCell;
            const std::size_t index = static_cast<std::size_t>(cell.row) * grid.getConfig().width
                + static_cast<std::size_t>(cell.col);
            const auto existing = evidenceByIndex.find(index);
            if (existing == evidenceByIndex.end() || occupied) {
                evidenceByIndex[index] = SlamCellEvidence{cell, occupied};
            }
        }
    }

    std::vector<SlamCellEvidence> evidence;
    evidence.reserve(evidenceByIndex.size());
    for (const auto& entry : evidenceByIndex) {
        evidence.push_back(entry.second);
        if (entry.second.occupied) {
            ++result.occupiedCells;
        } else {
            ++result.freeCells;
        }
    }
    result.integrated = grid.applyEvidence(evidence);
    return result;
}
