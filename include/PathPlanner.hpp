#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "MapData.hpp"

struct PathResult {
    bool success = false;
    std::vector<GridCoord> path;
    std::string message;
    int nodesExpanded = 0;
    float pathLength = 0.0f;
};

class PathPlanner {
public:
    static PathResult plan(const MapData& mapData);
    static PathResult plan(const MapData& mapData, float clearanceRadius);
    static PathResult plan(
        const MapData& mapData,
        const Pose2D& explicitStartPose,
        const Pose2D& explicitGoalPose,
        float clearanceRadius
    );

private:
    static PathResult planWithClearance(const MapData& mapData, float clearanceRadius);
    static std::optional<GridCoord> getStartCell(const MapData& mapData);
    static std::optional<GridCoord> getGoalCell(const MapData& mapData);
    static bool isPositionBlocked(
        const MapData& mapData,
        const sf::Vector2f& position,
        float clearanceRadius
    );
    static bool isCellBlocked(
        const MapData& mapData,
        const GridCoord& cell,
        float clearanceRadius
    );
    static std::vector<GridCoord> getNeighbors(
        const MapData& mapData,
        const GridCoord& cell,
        float clearanceRadius
    );
    static float heuristic(const GridCoord& from, const GridCoord& to);
    static std::vector<GridCoord> reconstructPath(
        const std::map<GridCoord, GridCoord>& cameFrom,
        const GridCoord& current
    );
};
