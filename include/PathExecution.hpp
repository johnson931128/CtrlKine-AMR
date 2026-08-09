#pragma once

#include <cstddef>
#include <optional>

#include "PathPlanner.hpp"

enum class PathExecutionState {
    NotFollowing,
    Following,
    Completed
};

class PathExecution {
public:
    void install(PathResult result);
    void clear();

    const PathResult& getResult() const;
    PathExecutionState getState() const;
    std::size_t getCurrentWaypointIndex() const;
    bool hasExecutablePath() const;
    std::optional<GridCoord> getCurrentWaypoint() const;

    void advanceWaypoint();

private:
    PathResult m_result;
    std::size_t m_currentWaypointIndex = 0;
    PathExecutionState m_state = PathExecutionState::NotFollowing;
};
