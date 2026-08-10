#include "PathPlanner.hpp"

#include <cmath>
#include <limits>
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

    runTest(suite, "CLEAR-001", "adjacent obstacle blocks an unsafe start center", [] {
        MapData map = makeMap(7, 7);
        setEndpoints(map, GridCoord{2, 3}, GridCoord{5, 3});
        map.addObstacle(GridCoord{3, 3});
        return isExplicitFailure(PathPlanner::plan(map, 6.0f), "start cell", true);
    });

    runTest(suite, "CLEAR-002", "wide opening remains traversable", [] {
        MapData map = makeMap(9, 7);
        setEndpoints(map, GridCoord{2, 3}, GridCoord{6, 3});
        for (int row : {0, 1, 5, 6}) {
            map.addObstacle(GridCoord{4, row});
        }
        const PathResult result = PathPlanner::plan(map, 6.0f);
        return isValidPath(result, GridCoord{2, 3}, GridCoord{6, 3}, map);
    });

    runTest(suite, "CLEAR-003", "narrow opening is rejected", [] {
        MapData map = makeMap(9, 7);
        setEndpoints(map, GridCoord{2, 3}, GridCoord{6, 3});
        for (int row : {0, 1, 2, 4, 5, 6}) {
            map.addObstacle(GridCoord{4, row});
        }
        const PathResult result = PathPlanner::plan(map, 6.0f);
        return isExplicitFailure(result, "no traversable path", false)
            && result.nodesExpanded > 0;
    });

    runTest(suite, "CLEAR-004", "goal footprint collision fails before search", [] {
        MapData map = makeMap(7, 7);
        setEndpoints(map, GridCoord{1, 3}, GridCoord{4, 3});
        map.addObstacle(GridCoord{3, 3});
        return isExplicitFailure(PathPlanner::plan(map, 6.0f), "goal cell", true);
    });

    runTest(suite, "CLEAR-004", "endpoint pose clearance is checked before its cell center", [] {
        MapData map = makeMap(7, 7);
        map.setRobotStartPose(Pose2D{sf::Vector2f(29.0f, 35.0f), 0.0f});
        map.setRobotGoalPose(Pose2D{
            map.getMapper().gridToWorldCenter(GridCoord{5, 3}),
            0.0f
        });
        map.addObstacle(GridCoord{3, 3});
        return isExplicitFailure(PathPlanner::plan(map, 4.0f), "start cell", true);
    });

    runTest(suite, "CLEAR-005", "world boundary is eroded by body clearance", [] {
        MapData map = makeMap(7, 7);
        setEndpoints(map, GridCoord{0, 3}, GridCoord{5, 3});
        return isExplicitFailure(PathPlanner::plan(map, 6.0f), "start cell", true);
    });

    runTest(suite, "CLEAR-006", "invalid clearance fails explicitly", [] {
        MapData map = makeMap(7, 7);
        setEndpoints(map, GridCoord{1, 3}, GridCoord{5, 3});
        const PathResult negative = PathPlanner::plan(map, -1.0f);
        const PathResult nonFinite = PathPlanner::plan(
            map,
            std::numeric_limits<float>::infinity()
        );
        return isExplicitFailure(negative, "clearance", true)
            && isExplicitFailure(nonFinite, "clearance", true);
    });

    runTest(suite, "PP-EXPLICIT-001", "explicit start plans without mutating persistent Start", [] {
        MapData map = makeMap(8, 4);
        const Pose2D persistentStart{map.getMapper().gridToWorldCenter(GridCoord{0, 1}), 0.4f};
        const Pose2D explicitStart{map.getMapper().gridToWorldCenter(GridCoord{2, 1}), -0.2f};
        const Pose2D goal{map.getMapper().gridToWorldCenter(GridCoord{6, 1}), 0.0f};
        map.setRobotStartPose(persistentStart);
        map.setRobotGoalPose(goal);
        const PathResult result = PathPlanner::plan(map, explicitStart, goal, 0.0f);
        return result.success && result.path.front() == GridCoord{2, 1}
            && map.getRobotStartPose()->position == persistentStart.position
            && map.getRobotStartPose()->heading == persistentStart.heading;
    });

    runTest(suite, "PP-EXPLICIT-002", "blocked and out-of-bound explicit starts fail", [] {
        MapData map = makeMap(8, 4);
        const Pose2D goal{map.getMapper().gridToWorldCenter(GridCoord{6, 1}), 0.0f};
        map.addObstacle(GridCoord{2, 1});
        const PathResult blocked = PathPlanner::plan(
            map, Pose2D{map.getMapper().gridToWorldCenter(GridCoord{2, 1}), 0.0f}, goal, 0.0f
        );
        const PathResult outside = PathPlanner::plan(
            map, Pose2D{sf::Vector2f(-10.0f, 15.0f), 0.0f}, goal, 0.0f
        );
        return isExplicitFailure(blocked, "start cell", true)
            && isExplicitFailure(outside, "start cell", true);
    });

    return suite.exitCode();
}
