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

private:
    static std::optional<GridCoord> getStartCell(const MapData& mapData);
    static std::optional<GridCoord> getGoalCell(const MapData& mapData);
    static bool isCellBlocked(const MapData& mapData, const GridCoord& cell);
    static std::vector<GridCoord> getNeighbors(const MapData& mapData, const GridCoord& cell);
    static float heuristic(const GridCoord& from, const GridCoord& to);
    static std::vector<GridCoord> reconstructPath(
        const std::map<GridCoord, GridCoord>& cameFrom,
        const GridCoord& current
    );
};
