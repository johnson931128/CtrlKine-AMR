#include "PathExecution.hpp"

#include <cmath>
#include <utility>

namespace {
bool isRedundantCollinearPoint(
    const GridCoord& previous,
    const GridCoord& current,
    const GridCoord& next
) {
    const int firstColStep = current.col - previous.col;
    const int firstRowStep = current.row - previous.row;
    const int secondColStep = next.col - current.col;
    const int secondRowStep = next.row - current.row;
    const bool firstIsUnitStep = std::abs(firstColStep) + std::abs(firstRowStep) == 1;
    const bool secondIsUnitStep = std::abs(secondColStep) + std::abs(secondRowStep) == 1;

    return firstIsUnitStep
        && secondIsUnitStep
        && firstColStep == secondColStep
        && firstRowStep == secondRowStep;
}

std::vector<GridCoord> makeExecutionWaypoints(const std::vector<GridCoord>& rawPath) {
    if (rawPath.size() <= 2) {
        return rawPath;
    }

    std::vector<GridCoord> waypoints;
    waypoints.push_back(rawPath.front());
    for (std::size_t index = 1; index + 1 < rawPath.size(); ++index) {
        if (!isRedundantCollinearPoint(rawPath[index - 1], rawPath[index], rawPath[index + 1])) {
            waypoints.push_back(rawPath[index]);
        }
    }
    waypoints.push_back(rawPath.back());
    return waypoints;
}
}

void PathExecution::install(PathResult result) {
    m_result = std::move(result);
    m_executionWaypoints = m_result.success
        ? makeExecutionWaypoints(m_result.path)
        : std::vector<GridCoord>{};
    m_currentWaypointIndex = 0;
    m_state = m_result.success && !m_executionWaypoints.empty()
        ? PathExecutionState::Following
        : PathExecutionState::NotFollowing;
}

void PathExecution::clear() {
    m_result = PathResult{};
    m_executionWaypoints.clear();
    m_currentWaypointIndex = 0;
    m_state = PathExecutionState::NotFollowing;
}

const PathResult& PathExecution::getResult() const {
    return m_result;
}

PathExecutionState PathExecution::getState() const {
    return m_state;
}

std::size_t PathExecution::getCurrentWaypointIndex() const {
    return m_currentWaypointIndex;
}

bool PathExecution::hasExecutablePath() const {
    return m_result.success && !m_executionWaypoints.empty();
}

std::optional<GridCoord> PathExecution::getCurrentWaypoint() const {
    if (m_state != PathExecutionState::Following
        || m_currentWaypointIndex >= m_executionWaypoints.size()) {
        return std::nullopt;
    }

    return m_executionWaypoints[m_currentWaypointIndex];
}

const std::vector<GridCoord>& PathExecution::getExecutionWaypoints() const {
    return m_executionWaypoints;
}

void PathExecution::advanceWaypoint() {
    if (m_state != PathExecutionState::Following
        || m_currentWaypointIndex >= m_executionWaypoints.size()) {
        return;
    }

    ++m_currentWaypointIndex;
    if (m_currentWaypointIndex >= m_executionWaypoints.size()) {
        m_state = PathExecutionState::Completed;
    }
}
