#include "Simulator.hpp"

#include <cmath>
#include <vector>

#include "TestSupport.hpp"

struct SimulatorRuntimeTestAccess {
    static bool applyConfiguredStartPose(
        const MapData& mapData,
        AMR& amr,
        PathExecution& pathExecution
    ) {
        return Simulator::applyConfiguredStartPose(mapData, amr, pathExecution);
    }

    static void applyRobotReset(
        const MapData& mapData,
        const sf::Vector2f& defaultPosition,
        AMR& amr,
        PathExecution& pathExecution
    ) {
        Simulator::applyRobotReset(mapData, defaultPosition, amr, pathExecution);
    }

    static bool isRobotAtInitialWaypoint(
        const MapData& mapData,
        const AMR& amr,
        const PathExecution& pathExecution
    ) {
        return Simulator::isRobotAtInitialWaypoint(mapData, amr, pathExecution);
    }
};

namespace {
constexpr float kHalfPi = 1.57079633f;

AMRConfig testConfig() {
    return AMRConfig{
        10.0f,
        20.0f,
        4.0f,
        2.0f,
        8.0f,
        12.0f,
        sf::Color::White,
        sf::Color::Black
    };
}

PathResult successfulResult(const std::vector<GridCoord>& path) {
    PathResult result;
    result.success = true;
    result.path = path;
    result.message = "success";
    return result;
}

bool near(float first, float second, float tolerance = 0.001f) {
    return std::abs(first - second) <= tolerance;
}

bool samePoint(const sf::Vector2f& first, const sf::Vector2f& second) {
    return near(first.x, second.x) && near(first.y, second.y);
}

bool executionIsCleared(const PathExecution& execution) {
    return execution.getState() == PathExecutionState::NotFollowing
        && execution.getCurrentWaypointIndex() == 0
        && !execution.hasExecutablePath()
        && !execution.getCurrentWaypoint().has_value()
        && execution.getResult().path.empty()
        && execution.getExecutionWaypoints().empty();
}
}

int main() {
    TestSuite suite;

    runTest(suite, "START-001", "configured Start synchronizes pose, geometry, and execution state", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(125.0f, 140.0f), kHalfPi};
        map.setRobotStartPose(startPose);

        AMR robot(testConfig(), sf::Vector2f(10.0f, 10.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        execution.advanceWaypoint();

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return applied
            && samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && robot.containsPoint(startPose.position + sf::Vector2f(0.0f, 9.0f))
            && !robot.containsPoint(startPose.position + sf::Vector2f(6.0f, 0.0f))
            && executionIsCleared(execution);
    });

    runTest(suite, "START-002", "changing Start replaces the runtime pose and resets progress", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        PathExecution execution;

        map.setRobotStartPose(Pose2D{sf::Vector2f(25.0f, 25.0f), 0.3f});
        SimulatorRuntimeTestAccess::applyConfiguredStartPose(map, robot, execution);

        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        execution.advanceWaypoint();
        const Pose2D changedStart{sf::Vector2f(175.0f, 85.0f), -0.7f};
        map.setRobotStartPose(changedStart);

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return applied
            && samePoint(robot.getPosition(), changedStart.position)
            && near(robot.getHeading(), changedStart.heading)
            && executionIsCleared(execution);
    });

    runTest(suite, "START-003", "missing Start does not alter runtime pose or execution", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(30.0f, 40.0f));
        robot.setPose(sf::Vector2f(30.0f, 40.0f), 0.5f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return !applied
            && samePoint(robot.getPosition(), sf::Vector2f(30.0f, 40.0f))
            && near(robot.getHeading(), 0.5f)
            && execution.getState() == PathExecutionState::Following
            && execution.getCurrentWaypointIndex() == 0;
    });

    runTest(suite, "START-004", "planning synchronization restores Start after manual runtime movement", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(75.0f, 125.0f), -0.4f};
        map.setRobotStartPose(startPose);
        AMR robot(testConfig(), sf::Vector2f(75.0f, 125.0f));
        robot.setPose(sf::Vector2f(300.0f, 250.0f), 1.0f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{6, 5}, GridCoord{5, 5}}));

        const bool applied = SimulatorRuntimeTestAccess::applyConfiguredStartPose(
            map, robot, execution
        );
        return applied
            && samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && executionIsCleared(execution);
    });

    runTest(suite, "RESET-001", "reset with Start restores its position and heading", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(-125.0f, 75.0f), -1.1f};
        map.setRobotStartPose(startPose);
        AMR robot(testConfig(), sf::Vector2f(300.0f, 300.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        SimulatorRuntimeTestAccess::applyRobotReset(
            map, sf::Vector2f(400.0f, 400.0f), robot, execution
        );
        return samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && executionIsCleared(execution);
    });

    runTest(suite, "RESET-002", "reset without Start restores the existing default pose", [] {
        MapData map(50.0f);
        const sf::Vector2f defaultPosition(400.0f, 400.0f);
        AMR robot(testConfig(), sf::Vector2f(20.0f, 30.0f));
        robot.setPose(sf::Vector2f(20.0f, 30.0f), 1.2f);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}}));

        SimulatorRuntimeTestAccess::applyRobotReset(
            map, defaultPosition, robot, execution
        );
        return samePoint(robot.getPosition(), defaultPosition)
            && near(robot.getHeading(), 0.0f)
            && executionIsCleared(execution);
    });

    runTest(suite, "START-005", "off-center Start is accepted as initial waypoint without approach motion", [] {
        MapData map(50.0f);
        const Pose2D startPose{sf::Vector2f(10.0f, 20.0f), 0.8f};
        map.setRobotStartPose(startPose);
        AMR robot(testConfig(), sf::Vector2f(300.0f, 300.0f));
        robot.setPose(startPose.position, startPose.heading);
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        const bool reached = SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(
            map, robot, execution
        );
        if (reached) {
            execution.advanceWaypoint();
        }
        return reached
            && samePoint(robot.getPosition(), startPose.position)
            && near(robot.getHeading(), startPose.heading)
            && execution.getCurrentWaypointIndex() == 1
            && execution.getState() == PathExecutionState::Following;
    });

    runTest(suite, "START-006", "same-cell shortcut applies only to waypoint zero", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(10.0f, 20.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        if (!SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(map, robot, execution)) {
            return false;
        }
        execution.advanceWaypoint();
        robot.setPose(sf::Vector2f(60.0f, 20.0f), 0.0f);
        return !SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(map, robot, execution);
    });

    runTest(suite, "START-007", "different-cell runtime pose is not the initial waypoint", [] {
        MapData map(50.0f);
        AMR robot(testConfig(), sf::Vector2f(60.0f, 20.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        return !SimulatorRuntimeTestAccess::isRobotAtInitialWaypoint(map, robot, execution)
            && execution.getCurrentWaypointIndex() == 0;
    });

    return suite.exitCode();
}
