#include "AMR.hpp"
#include "PathExecution.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "TestSupport.hpp"

namespace {
PathResult successfulResult(const std::vector<GridCoord>& path) {
    PathResult result;
    result.success = true;
    result.path = path;
    result.message = "success";
    return result;
}

AMRConfig testConfig() {
    return AMRConfig{
        20.0f,
        10.0f,
        4.0f,
        2.0f,
        8.0f,
        12.0f,
        sf::Color::White,
        sf::Color::Black
    };
}

bool near(float first, float second, float tolerance = 0.001f) {
    return std::abs(first - second) <= tolerance;
}

bool samePoint(const sf::Vector2f& point, float x, float y) {
    return near(point.x, x) && near(point.y, y);
}

bool isWaypoint(const PathExecution& execution, const GridCoord& expected) {
    const std::optional<GridCoord> waypoint = execution.getCurrentWaypoint();
    return waypoint.has_value() && *waypoint == expected;
}
}

int main() {
    TestSuite suite;

    runTest(suite, "EXEC-001", "default and empty execution are safe", [] {
        PathExecution execution;
        execution.advanceWaypoint();
        execution.install(successfulResult({}));
        return execution.getState() == PathExecutionState::NotFollowing
            && !execution.hasExecutablePath()
            && !execution.getCurrentWaypoint().has_value()
            && execution.getCurrentWaypointIndex() == 0;
    });

    runTest(suite, "EXEC-002", "successful install retains and exposes the first waypoint", [] {
        const std::vector<GridCoord> path = {
            GridCoord{1, 2},
            GridCoord{2, 2}
        };
        PathExecution execution;
        execution.install(successfulResult(path));
        return execution.getState() == PathExecutionState::Following
            && execution.hasExecutablePath()
            && execution.getResult().path == path
            && execution.getCurrentWaypoint() == std::optional<GridCoord>(path.front());
    });

    runTest(suite, "EXEC-003", "failed replacement cannot begin or retain execution", [] {
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));

        PathResult failure;
        failure.message = "planning failed";
        execution.install(failure);

        return execution.getState() == PathExecutionState::NotFollowing
            && !execution.hasExecutablePath()
            && execution.getResult().path.empty()
            && !execution.getCurrentWaypoint().has_value();
    });

    runTest(suite, "EXEC-004", "waypoints advance in order and complete at the final cell", [] {
        const std::vector<GridCoord> path = {
            GridCoord{0, 0},
            GridCoord{1, 0},
            GridCoord{1, 1}
        };
        PathExecution execution;
        execution.install(successfulResult(path));

        if (!isWaypoint(execution, path[0])) {
            return false;
        }
        execution.advanceWaypoint();
        if (!isWaypoint(execution, path[1])) {
            return false;
        }
        execution.advanceWaypoint();
        if (!isWaypoint(execution, path[2])) {
            return false;
        }
        execution.advanceWaypoint();

        return execution.getState() == PathExecutionState::Completed
            && execution.getCurrentWaypointIndex() == path.size()
            && !execution.getCurrentWaypoint().has_value();
    });

    runTest(suite, "EXEC-005", "single-cell path completes without out-of-range access", [] {
        PathExecution execution;
        execution.install(successfulResult({GridCoord{4, 3}}));
        execution.advanceWaypoint();
        execution.advanceWaypoint();
        return execution.getState() == PathExecutionState::Completed
            && execution.getCurrentWaypointIndex() == 1
            && !execution.getCurrentWaypoint().has_value();
    });

    runTest(suite, "EXEC-006", "path replacement resets progress to the new first waypoint", [] {
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}, GridCoord{1, 0}}));
        execution.advanceWaypoint();

        const GridCoord replacementStart{8, 7};
        execution.install(successfulResult({replacementStart, GridCoord{8, 8}}));
        return execution.getState() == PathExecutionState::Following
            && execution.getCurrentWaypointIndex() == 0
            && execution.getCurrentWaypoint() == std::optional<GridCoord>(replacementStart);
    });

    runTest(suite, "EXEC-007", "clear removes executable and visualized path data", [] {
        PathExecution execution;
        execution.install(successfulResult({GridCoord{2, 2}}));
        execution.clear();
        return execution.getState() == PathExecutionState::NotFollowing
            && !execution.hasExecutablePath()
            && execution.getResult().path.empty()
            && !execution.getCurrentWaypoint().has_value();
    });

    runTest(suite, "EXEC-008", "collinear cells become endpoint execution waypoints", [] {
        const std::vector<GridCoord> rawPath = {
            GridCoord{0, 0},
            GridCoord{1, 0},
            GridCoord{2, 0},
            GridCoord{3, 0}
        };
        PathExecution execution;
        execution.install(successfulResult(rawPath));
        const std::vector<GridCoord> expectedWaypoints = {
            GridCoord{0, 0},
            GridCoord{3, 0}
        };
        return execution.getResult().path == rawPath
            && execution.getExecutionWaypoints() == expectedWaypoints;
    });

    runTest(suite, "EXEC-009", "required corners remain execution waypoints", [] {
        PathExecution execution;
        execution.install(successfulResult({
            GridCoord{0, 0},
            GridCoord{1, 0},
            GridCoord{2, 0},
            GridCoord{2, 1},
            GridCoord{2, 2}
        }));
        const std::vector<GridCoord> expectedWaypoints = {
            GridCoord{0, 0},
            GridCoord{2, 0},
            GridCoord{2, 2}
        };
        return execution.getExecutionWaypoints() == expectedWaypoints;
    });

    runTest(suite, "EXEC-010", "direction changes and invalid gaps are not simplified", [] {
        const std::vector<GridCoord> rawPath = {
            GridCoord{0, 0},
            GridCoord{1, 0},
            GridCoord{0, 0},
            GridCoord{2, 0}
        };
        PathExecution execution;
        execution.install(successfulResult(rawPath));
        return execution.getExecutionWaypoints() == rawPath;
    });

    runTest(suite, "MOVE-001", "moveToward clamps travel and does not overshoot", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        const bool reached = robot.moveToward(sf::Vector2f(5.0f, 0.0f), 1.0f, 10.0f);
        return reached && samePoint(robot.getPosition(), 5.0f, 0.0f);
    });

    runTest(suite, "MOVE-002", "moveToward remains frame-time independent", [] {
        AMR oneStep(testConfig(), sf::Vector2f(0.0f, 0.0f));
        AMR twoSteps(testConfig(), sf::Vector2f(0.0f, 0.0f));
        const sf::Vector2f target(20.0f, 0.0f);

        oneStep.moveToward(target, 1.0f, 10.0f);
        twoSteps.moveToward(target, 0.5f, 10.0f);
        twoSteps.moveToward(target, 0.5f, 10.0f);

        return samePoint(oneStep.getPosition(), 10.0f, 0.0f)
            && samePoint(twoSteps.getPosition(), 10.0f, 0.0f);
    });

    runTest(suite, "MOVE-003", "moveToward snaps safely within arrival tolerance", [] {
        AMR robot(testConfig(), sf::Vector2f(5.0f, 5.0f));
        const bool reached = robot.moveToward(sf::Vector2f(5.2f, 5.2f), 0.0f, 0.0f, 0.5f);
        return reached && samePoint(robot.getPosition(), 5.2f, 5.2f);
    });

    runTest(suite, "MOVE-004", "moveToward points the robot toward the waypoint", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        robot.moveToward(sf::Vector2f(0.0f, 10.0f), 0.5f, 10.0f);
        return samePoint(robot.getPosition(), 0.0f, 5.0f)
            && near(robot.getHeading(), 1.57079633f);
    });

    runTest(suite, "MOVE-005", "automatic heading change is bounded per update", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        const bool reached = robot.moveToward(
            sf::Vector2f(0.0f, 10.0f),
            0.1f,
            10.0f,
            1.0f,
            0.5f
        );
        return !reached
            && samePoint(robot.getPosition(), 0.0f, 0.0f)
            && near(robot.getHeading(), 0.1f);
    });

    runTest(suite, "MOVE-006", "sharp automatic turn does not snap heading", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        robot.moveToward(sf::Vector2f(0.0f, 10.0f), 0.25f, 10.0f, 1.0f, 0.5f);
        return robot.getHeading() > 0.0f
            && robot.getHeading() < 1.57079633f
            && samePoint(robot.getPosition(), 0.0f, 0.0f);
    });

    runTest(suite, "MOVE-007", "aligned automatic motion progresses and completes", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        PathExecution execution;
        execution.install(successfulResult({GridCoord{0, 0}}));
        const bool reached = robot.moveToward(
            sf::Vector2f(5.0f, 0.0f),
            1.0f,
            10.0f,
            1.0f,
            0.5f
        );
        if (reached) {
            execution.advanceWaypoint();
        }
        return reached
            && samePoint(robot.getPosition(), 5.0f, 0.0f)
            && execution.getState() == PathExecutionState::Completed;
    });

    runTest(suite, "MOVE-008", "invalid automatic update inputs leave pose unchanged", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        const float infinity = std::numeric_limits<float>::infinity();
        const bool zeroDt = robot.moveToward(
            sf::Vector2f(10.0f, 0.0f), 0.0f, 10.0f, 1.0f, 0.5f
        );
        const bool invalidRate = robot.moveToward(
            sf::Vector2f(10.0f, 0.0f), 0.1f, 10.0f, infinity, 0.5f
        );
        return !zeroDt && !invalidRate
            && samePoint(robot.getPosition(), 0.0f, 0.0f)
            && near(robot.getHeading(), 0.0f);
    });

    runTest(suite, "MOVE-009", "body clearance radius encloses every heading", [] {
        AMR robot(testConfig(), sf::Vector2f(0.0f, 0.0f));
        return near(robot.getConservativeBodyRadius(), std::hypot(5.0f, 10.0f));
    });

    return suite.exitCode();
}
