#pragma once

#include <cstddef>
#include <optional>

#include "core/CoordinateTypes.hpp"

enum class SelectedObjectType {
    None,
    Obstacle,
    WorkZone,
    StartPose,
    GoalPose,
    Robot
};

struct SelectedObject {
    SelectedObjectType type = SelectedObjectType::None;
    std::optional<GridCoord> obstacleCoord;
    std::optional<std::size_t> workZoneIndex;

    static SelectedObject none() {
        return SelectedObject{};
    }

    static SelectedObject obstacle(const GridCoord& coord) {
        SelectedObject selected;
        selected.type = SelectedObjectType::Obstacle;
        selected.obstacleCoord = coord;
        return selected;
    }

    static SelectedObject workZone(std::size_t index) {
        SelectedObject selected;
        selected.type = SelectedObjectType::WorkZone;
        selected.workZoneIndex = index;
        return selected;
    }

    static SelectedObject startPose() {
        SelectedObject selected;
        selected.type = SelectedObjectType::StartPose;
        return selected;
    }

    static SelectedObject goalPose() {
        SelectedObject selected;
        selected.type = SelectedObjectType::GoalPose;
        return selected;
    }

    static SelectedObject robot() {
        SelectedObject selected;
        selected.type = SelectedObjectType::Robot;
        return selected;
    }

    bool isNone() const {
        return type == SelectedObjectType::None;
    }
};
