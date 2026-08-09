#include "MapData.hpp"

#include <cmath>

#include "TestSupport.hpp"

namespace {
bool sameFloat(float lhs, float rhs) { return std::fabs(lhs - rhs) < 0.0001f; }
bool samePoint(const sf::Vector2f& lhs, const sf::Vector2f& rhs) {
    return sameFloat(lhs.x, rhs.x) && sameFloat(lhs.y, rhs.y);
}
bool sameRect(const sf::FloatRect& lhs, const sf::FloatRect& rhs) {
    return samePoint(lhs.position, rhs.position) && samePoint(lhs.size, rhs.size);
}
}

int main() {
    TestSuite suite;
    MapData map(50.0f);
    map.setWorldBoundary(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(100.0f, 100.0f)));

    runTest(suite, "MAP-001", "default world boundary", [] {
        const MapData defaultMap;
        const auto& boundary = defaultMap.getWorldBoundary();
        return samePoint(boundary.position, sf::Vector2f(-1000.0f, -1000.0f))
            && samePoint(boundary.size, sf::Vector2f(2000.0f, 2000.0f));
    });
    runTest(suite, "MAP-002", "half-open boundary containment", [&] {
        return map.containsWorldPoint(sf::Vector2f(0.0f, 0.0f))
            && map.containsWorldPoint(sf::Vector2f(99.999f, 99.999f))
            && !map.containsWorldPoint(sf::Vector2f(100.0f, 50.0f))
            && !map.containsWorldPoint(sf::Vector2f(50.0f, 100.0f))
            && !map.containsWorldPoint(sf::Vector2f(-0.001f, 50.0f));
    });
    runTest(suite, "MAP-003", "add obstacle by world position", [&] {
        map.clear();
        map.addObstacle(sf::Vector2f(25.0f, 25.0f));
        const bool accepted = map.isObstacleAt(GridCoord{0, 0});
        map.addObstacle(sf::Vector2f(100.0f, 25.0f));
        return accepted && map.getObstacles().size() == 1;
    });
    runTest(suite, "MAP-003", "world position uses point boundary check", [] {
        MapData partialCellMap(50.0f);
        partialCellMap.setWorldBoundary(
            sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(20.0f, 20.0f)));
        partialCellMap.addObstacle(sf::Vector2f(10.0f, 10.0f));
        return partialCellMap.isObstacleAt(GridCoord{0, 0});
    });
    runTest(suite, "MAP-003", "add obstacle by GridCoord with boundary check", [&] {
        map.clear();
        map.addObstacle(GridCoord{1, 1});
        map.addObstacle(GridCoord{2, 2});
        return map.isObstacleAt(GridCoord{1, 1})
            && !map.isObstacleAt(GridCoord{2, 2})
            && map.getObstacles().size() == 1;
    });
    runTest(suite, "MAP-004", "duplicate obstacle", [&] {
        map.clear();
        map.addObstacle(sf::Vector2f(25.0f, 25.0f));
        map.addObstacle(GridCoord{0, 0});
        return map.getObstacles().size() == 1;
    });
    runTest(suite, "MAP-005", "remove existing and non-existing obstacle", [&] {
        map.clear();
        map.addObstacle(GridCoord{1, 1});
        map.removeObstacle(GridCoord{1, 1});
        const std::size_t afterExisting = map.getObstacles().size();
        map.removeObstacle(GridCoord{9, 9});
        return afterExisting == 0 && map.getObstacles().empty();
    });
    runTest(suite, "MAP-006", "obstacle lookup by world and grid coordinate", [&] {
        map.clear();
        map.addObstacle(GridCoord{1, 0});
        return map.isObstacleAt(GridCoord{1, 0})
            && map.isObstacleAt(sf::Vector2f(75.0f, 25.0f));
    });
    runTest(suite, "MAP-007", "start pose set, read, clear", [&] {
        const Pose2D pose{sf::Vector2f(25.0f, 25.0f), 0.75f};
        map.setRobotStartPose(pose);
        const bool stored = map.getRobotStartPose().has_value()
            && samePoint(map.getRobotStartPose()->position, pose.position)
            && sameFloat(map.getRobotStartPose()->heading, pose.heading);
        map.clearRobotStartPose();
        return stored && !map.getRobotStartPose().has_value();
    });
    runTest(suite, "MAP-008", "goal pose set, read, clear", [&] {
        const Pose2D pose{sf::Vector2f(75.0f, 75.0f), 1.25f};
        map.setRobotGoalPose(pose);
        const bool stored = map.getRobotGoalPose().has_value()
            && samePoint(map.getRobotGoalPose()->position, pose.position)
            && sameFloat(map.getRobotGoalPose()->heading, pose.heading);
        map.clearRobotGoalPose();
        return stored && !map.getRobotGoalPose().has_value();
    });
    runTest(suite, "MAP-009", "valid and invalid work zones", [&] {
        map.clear();
        map.addWorkZone(sf::FloatRect(sf::Vector2f(10.0f, 10.0f), sf::Vector2f(20.0f, 30.0f)));
        map.addWorkZone(sf::FloatRect(sf::Vector2f(10.0f, 10.0f), sf::Vector2f(0.0f, 30.0f)));
        map.addWorkZone(sf::FloatRect(sf::Vector2f(10.0f, 10.0f), sf::Vector2f(-1.0f, 30.0f)));
        map.addWorkZone(sf::FloatRect(sf::Vector2f(90.0f, 90.0f), sf::Vector2f(20.0f, 20.0f)));
        return map.getWorkZones().size() == 1
            && sameRect(map.getWorkZones().front().bounds,
                        sf::FloatRect(sf::Vector2f(10.0f, 10.0f), sf::Vector2f(20.0f, 30.0f)));
    });
    runTest(suite, "MAP-010", "remove valid and invalid work-zone index", [&] {
        map.clear();
        map.addWorkZone(sf::FloatRect(sf::Vector2f(10.0f, 10.0f), sf::Vector2f(20.0f, 20.0f)));
        map.addWorkZone(sf::FloatRect(sf::Vector2f(40.0f, 40.0f), sf::Vector2f(20.0f, 20.0f)));
        const bool removed = map.removeWorkZone(0);
        const bool invalid = !map.removeWorkZone(99);
        return removed && invalid && map.getWorkZones().size() == 1
            && samePoint(map.getWorkZones().front().bounds.position, sf::Vector2f(40.0f, 40.0f));
    });
    runTest(suite, "MAP-011", "clear editable map data", [&] {
        map.clear();
        map.addObstacle(GridCoord{0, 0});
        map.addWorkZone(sf::FloatRect(sf::Vector2f(10.0f, 10.0f), sf::Vector2f(20.0f, 20.0f)));
        map.setRobotStartPose(Pose2D{sf::Vector2f(25.0f, 25.0f), 0.0f});
        map.setRobotGoalPose(Pose2D{sf::Vector2f(75.0f, 75.0f), 1.0f});
        map.clear();
        return map.getObstacles().empty() && map.getWorkZones().empty()
            && !map.getRobotStartPose().has_value() && !map.getRobotGoalPose().has_value();
    });
    runTest(suite, "COORD-005", "MapData rejects non-positive resolution", [] {
        MapData data(50.0f);
        data.setGridResolution(0.0f);
        const bool zeroRejected = data.getGridResolution() == 50.0f;
        data.setGridResolution(-5.0f);
        return zeroRejected && data.getGridResolution() == 50.0f;
    });
    runTest(suite, "COORD-005", "MapData constructor rejects non-positive resolution", [] {
        return MapData(0.0f).getGridResolution() > 0.0f
            && MapData(-1.0f).getGridResolution() > 0.0f;
    });

    return suite.exitCode();
}
