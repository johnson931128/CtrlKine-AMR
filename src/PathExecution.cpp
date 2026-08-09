#include "PathExecution.hpp"

#include <utility>

void PathExecution::install(PathResult result) {
    m_result = std::move(result);
    m_currentWaypointIndex = 0;
    m_state = m_result.success && !m_result.path.empty()
        ? PathExecutionState::Following
        : PathExecutionState::NotFollowing;
}

void PathExecution::clear() {
    m_result = PathResult{};
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
    return m_result.success && !m_result.path.empty();
}

std::optional<GridCoord> PathExecution::getCurrentWaypoint() const {
    if (m_state != PathExecutionState::Following
        || m_currentWaypointIndex >= m_result.path.size()) {
        return std::nullopt;
    }

    return m_result.path[m_currentWaypointIndex];
}

void PathExecution::advanceWaypoint() {
    if (m_state != PathExecutionState::Following
        || m_currentWaypointIndex >= m_result.path.size()) {
        return;
    }

    ++m_currentWaypointIndex;
    if (m_currentWaypointIndex >= m_result.path.size()) {
        m_state = PathExecutionState::Completed;
    }
}
