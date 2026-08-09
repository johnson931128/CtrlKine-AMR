#include "PathPlanner.hpp"

#include <cmath>
#include <string>

#include "TestSupport.hpp"

namespace {
MapData makeMap(int width, int height) {
    MapData map(10.0f);
    map.setWorldBoundary(sf::FloatRect(
        sf::Vector2f(0.0f, 0.0f),
        sf::Vector2f(width * 10.0f, height * 10.0f)
    ));
    return map;
}

void setEndpoints(MapData& map, const GridCoord& start, const GridCoord& goal) {
    map.setRobotStartPose(Pose2D{map.getMapper().gridToWorldCenter(start), 0.0f});
    map.setRobotGoalPose(Pose2D{map.getMapper().gridToWorldCenter(goal), 1.0f});
}

bool hasMessage(const PathResult& result, const std::string& text) {
    return !result.message.empty() && result.message.find(text) != std::string::npos;
}

bool isFourNeighbor(const GridCoord& first, const GridCoord& second) {
    return std::abs(first.col - second.col) + std::abs(first.row - second.row) == 1;
}

bool isValidPath(
    const PathResult& result,
    const GridCoord& start,
    const GridCoord& goal,
    const MapData& map
) {
    if (!result.success || result.path.empty()
        || !(result.path.front() == start) || !(result.path.back() == goal)
        || result.message.empty() || result.nodesExpanded < 1
        || result.pathLength != static_cast<float>(result.path.size() - 1)) {
        return false;
    }

    for (std::size_t index = 0; index < result.path.size(); ++index) {
        if (map.isObstacleAt(result.path[index])) {
            return false;
        }
        if (index > 0 && !isFourNeighbor(result.path[index - 1], result.path[index])) {
            return false;
        }
    }

    return true;
}

bool isExplicitFailure(
    const PathResult& result,
    const std::string& messageCategory,
    bool requireZeroExpanded
) {
    return !result.success
        && result.path.empty()
        && hasMessage(result, messageCategory)
        && (!requireZeroExpanded || result.nodesExpanded == 0)
        && result.nodesExpanded >= 0
        && result.pathLength == 0.0f;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "PP-002/PP-013/PP-014/PP-015/PP-016/PP-017/PP-018",
            "missing start pose returns explicit failure", [] {
        MapData map = makeMap(3, 1);
        map.setRobotGoalPose(Pose2D{map.getMapper().gridToWorldCenter(GridCoord{2, 0}), 0.0f});
        return isExplicitFailure(PathPlanner::plan(map), "start pose", true);
    });

    runTest(suite, "PP-003/PP-013/PP-014/PP-015/PP-016/PP-017/PP-018",
            "missing goal pose returns explicit failure", [] {
        MapData map = makeMap(3, 1);
        map.setRobotStartPose(Pose2D{map.getMapper().gridToWorldCenter(GridCoord{0, 0}), 0.0f});
        return isExplicitFailure(PathPlanner::plan(map), "goal pose", true);
    });

    runTest(suite, "PP-012/PP-013/PP-014/PP-015/PP-016/PP-017/PP-019",
            "blocked start cell returns explicit failure", [] {
        MapData map = makeMap(3, 1);
        setEndpoints(map, GridCoord{0, 0}, GridCoord{2, 0});
        map.addObstacle(GridCoord{0, 0});
        return isExplicitFailure(PathPlanner::plan(map), "start cell", true);
    });

    runTest(suite, "PP-012/PP-013/PP-014/PP-015/PP-016/PP-017/PP-019",
            "blocked goal cell returns explicit failure", [] {
        MapData map = makeMap(3, 1);
        setEndpoints(map, GridCoord{0, 0}, GridCoord{2, 0});
        map.addObstacle(GridCoord{2, 0});
        return isExplicitFailure(PathPlanner::plan(map), "goal cell", true);
    });

    runTest(suite, "PP-004/PP-013/PP-014/PP-015/PP-016/PP-017",
            "same traversable cell returns one-cell zero-length path", [] {
        MapData map = makeMap(2, 2);
        const GridCoord cell{1, 0};
        setEndpoints(map, cell, cell);
        const PathResult result = PathPlanner::plan(map);
        return result.success
            && result.path.size() == 1
            && result.path.front() == cell
            && result.message.size() > 0
            && result.nodesExpanded == 1
            && result.pathLength == 0.0f;
    });

    runTest(suite, "PP-006/PP-008/PP-013/PP-014/PP-015/PP-016/PP-017",
            "simple reachable path is valid from start to goal", [] {
        MapData map = makeMap(3, 1);
        const GridCoord start{0, 0};
        const GridCoord goal{2, 0};
        setEndpoints(map, start, goal);
        return isValidPath(PathPlanner::plan(map), start, goal, map);
    });

    runTest(suite, "PP-013/PP-014/PP-015/PP-016/PP-017/PP-020",
            "no traversable path returns explicit failure", [] {
        MapData map = makeMap(3, 3);
        setEndpoints(map, GridCoord{0, 0}, GridCoord{2, 2});
        map.addObstacle(GridCoord{1, 0});
        map.addObstacle(GridCoord{0, 1});
        const PathResult result = PathPlanner::plan(map);
        return isExplicitFailure(result, "no traversable path", false)
            && result.nodesExpanded > 0;
    });

    return suite.exitCode();
}
