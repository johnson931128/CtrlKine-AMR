#include "navigation/PathPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <set>

PathResult PathPlanner::plan(const MapData& mapData) {
    return planWithClearance(mapData, 0.0f);
}

PathResult PathPlanner::plan(const MapData& mapData, float clearanceRadius) {
    if (!std::isfinite(clearanceRadius) || clearanceRadius < 0.0f) {
        PathResult result;
        result.message = "Planning failed: clearance radius is invalid.";
        return result;
    }

    return planWithClearance(mapData, clearanceRadius);
}

PathResult PathPlanner::plan(
    const MapData& mapData,
    const Pose2D& explicitStartPose,
    const Pose2D& explicitGoalPose,
    float clearanceRadius
) {
    if (!std::isfinite(explicitStartPose.position.x)
        || !std::isfinite(explicitStartPose.position.y)
        || !std::isfinite(explicitStartPose.heading)
        || !std::isfinite(explicitGoalPose.position.x)
        || !std::isfinite(explicitGoalPose.position.y)
        || !std::isfinite(explicitGoalPose.heading)
        || !std::isfinite(clearanceRadius) || clearanceRadius < 0.0f) {
        PathResult result;
        result.message = "Planning failed: explicit pose or clearance is invalid.";
        return result;
    }
    if (!mapData.containsWorldPoint(explicitStartPose.position)) {
        PathResult result;
        result.message = "Planning failed: start cell is blocked.";
        return result;
    }
    if (!mapData.containsWorldPoint(explicitGoalPose.position)) {
        PathResult result;
        result.message = "Planning failed: goal cell is blocked.";
        return result;
    }
    MapData planningMap = mapData;
    planningMap.setRobotStartPose(explicitStartPose);
    planningMap.setRobotGoalPose(explicitGoalPose);
    return planWithClearance(planningMap, clearanceRadius);
}

PathResult PathPlanner::planWithClearance(const MapData& mapData, float clearanceRadius) {
    PathResult result;

    const std::optional<GridCoord> startCell = getStartCell(mapData);
    if (!startCell.has_value()) {
        result.message = "Planning failed: start pose is missing.";
        return result;
    }

    const std::optional<GridCoord> goalCell = getGoalCell(mapData);
    if (!goalCell.has_value()) {
        result.message = "Planning failed: goal pose is missing.";
        return result;
    }

    const bool startPoseBlocked = clearanceRadius > 0.0f
        && isPositionBlocked(mapData, mapData.getRobotStartPose()->position, clearanceRadius);
    if (startPoseBlocked || isCellBlocked(mapData, *startCell, clearanceRadius)) {
        result.message = "Planning failed: start cell is blocked.";
        return result;
    }

    const bool goalPoseBlocked = clearanceRadius > 0.0f
        && isPositionBlocked(mapData, mapData.getRobotGoalPose()->position, clearanceRadius);
    if (goalPoseBlocked || isCellBlocked(mapData, *goalCell, clearanceRadius)) {
        result.message = "Planning failed: goal cell is blocked.";
        return result;
    }

    std::set<GridCoord> openSet;
    std::map<GridCoord, GridCoord> cameFrom;
    std::map<GridCoord, float> gScore;
    std::map<GridCoord, float> fScore;

    gScore[*startCell] = 0.0f;
    fScore[*startCell] = heuristic(*startCell, *goalCell);
    openSet.insert(*startCell);

    while (!openSet.empty()) {
        auto currentIt = openSet.begin();
        for (auto candidateIt = openSet.begin(); candidateIt != openSet.end(); ++candidateIt) {
            if (fScore.at(*candidateIt) < fScore.at(*currentIt)) {
                currentIt = candidateIt;
            }
        }

        const GridCoord current = *currentIt;
        openSet.erase(currentIt);
        ++result.nodesExpanded;

        if (current == *goalCell) {
            result.path = reconstructPath(cameFrom, current);
            result.success = true;
            result.pathLength = static_cast<float>(result.path.size() - 1);
            result.message = "Path planning succeeded.";
            return result;
        }

        const float tentativeBaseGScore = gScore.at(current) + 1.0f;
        for (const GridCoord& neighbor : getNeighbors(mapData, current, clearanceRadius)) {
            const auto knownGScore = gScore.find(neighbor);
            if (knownGScore != gScore.end() && tentativeBaseGScore >= knownGScore->second) {
                continue;
            }

            cameFrom[neighbor] = current;
            gScore[neighbor] = tentativeBaseGScore;
            fScore[neighbor] = tentativeBaseGScore + heuristic(neighbor, *goalCell);
            openSet.insert(neighbor);
        }
    }

    result.success = false;
    result.path.clear();
    result.pathLength = 0.0f;
    result.message = "Planning failed: no traversable path exists.";

    return result;
}

std::optional<GridCoord> PathPlanner::getStartCell(const MapData& mapData) {
    // TODO(student):
    // 1. 從 MapData 取得 StartPose。
    // 2. 將 StartPose 的 world position 轉成 GridCoord。
    // 3. 若 StartPose 不存在，回傳 std::nullopt。
    if (!mapData.getRobotStartPose().has_value()) {
        return std::nullopt;
    }

    return mapData.getMapper().worldToGrid(mapData.getRobotStartPose()->position);
}

std::optional<GridCoord> PathPlanner::getGoalCell(const MapData& mapData) {
    // TODO(student):
    // 1. 從 MapData 取得 GoalPose。
    // 2. 將 GoalPose 的 world position 轉成 GridCoord。
    // 3. 若 GoalPose 不存在，回傳 std::nullopt。
    if (!mapData.getRobotGoalPose().has_value()) {
        return std::nullopt;
    }

    return mapData.getMapper().worldToGrid(mapData.getRobotGoalPose()->position);
}

bool PathPlanner::isPositionBlocked(
    const MapData& mapData,
    const sf::Vector2f& position,
    float clearanceRadius
) {
    if (clearanceRadius == 0.0f) {
        return !mapData.containsWorldPoint(position) || mapData.isObstacleAt(position);
    }

    const sf::FloatRect& boundary = mapData.getWorldBoundary();
    const float right = boundary.position.x + boundary.size.x;
    const float bottom = boundary.position.y + boundary.size.y;
    if (position.x - clearanceRadius < boundary.position.x
        || position.y - clearanceRadius < boundary.position.y
        || position.x + clearanceRadius >= right
        || position.y + clearanceRadius >= bottom) {
        return true;
    }

    const float radiusSquared = clearanceRadius * clearanceRadius;
    const float gridSize = mapData.getGridResolution();
    for (const GridCoord& obstacle : mapData.getObstacles()) {
        const sf::Vector2f topLeft = mapData.getMapper().gridToWorldTopLeft(obstacle);
        const float closestX = std::clamp(position.x, topLeft.x, topLeft.x + gridSize);
        const float closestY = std::clamp(position.y, topLeft.y, topLeft.y + gridSize);
        const float dx = position.x - closestX;
        const float dy = position.y - closestY;
        if ((dx * dx) + (dy * dy) <= radiusSquared) {
            return true;
        }
    }

    return false;
}

bool PathPlanner::isCellBlocked(
    const MapData& mapData,
    const GridCoord& cell,
    float clearanceRadius
) {
    // Planning is center-based, so cells and endpoint poses share the same
    // world-space clearance predicate.
    const sf::Vector2f cellCenter = mapData.getMapper().gridToWorldCenter(cell);
    return isPositionBlocked(mapData, cellCenter, clearanceRadius);
}

std::vector<GridCoord> PathPlanner::getNeighbors(
    const MapData& mapData,
    const GridCoord& cell,
    float clearanceRadius
) {
    // TODO(student):
    // 1. 先決定要用 4-neighbor 還是 8-neighbor。
    // 2. 產生 candidate cells。
    // 3. 過濾掉 blocked cell。
    // 4. 若之後要加入 corner cutting 規則，可以在這裡處理。
    const std::vector<GridCoord> candidateCells = {
        GridCoord{cell.col + 1, cell.row},
        GridCoord{cell.col - 1, cell.row},
        GridCoord{cell.col, cell.row + 1},
        GridCoord{cell.col, cell.row - 1}
    };

    std::vector<GridCoord> neighbors;
    for (const auto& candidate : candidateCells) {
        if (!isCellBlocked(mapData, candidate, clearanceRadius)) {
            neighbors.push_back(candidate);
        }
    }

    return neighbors;
}

float PathPlanner::heuristic(const GridCoord& from, const GridCoord& to) {
    // TODO(student):
    // 1. 依照你的鄰居定義選擇 heuristic。
    // 2. 若用 4-neighbor，常見選擇是 Manhattan distance。
    // 3. 若用 8-neighbor，常見選擇是 Octile distance 或 Euclidean distance。
    return static_cast<float>(std::abs(from.col - to.col) + std::abs(from.row - to.row));
}

std::vector<GridCoord> PathPlanner::reconstructPath(
    const std::map<GridCoord, GridCoord>& cameFrom,
    const GridCoord& current
) {
    // TODO(student):
    // 1. 從 goal 開始，沿著 cameFrom 一路回推到 start。
    // 2. 將回推出來的節點順序反轉，變成 start -> goal。
    // 3. 回傳完整 path，之後 Simulator 才能畫 path polyline。
    std::vector<GridCoord> path;
    GridCoord cursor = current;

    path.push_back(cursor);
    while (true) {
        const auto it = cameFrom.find(cursor);
        if (it == cameFrom.end()) {
            break;
        }

        cursor = it->second;
        path.push_back(cursor);
    }

    std::reverse(path.begin(), path.end());
    return path;
}
