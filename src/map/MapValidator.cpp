#include "map/MapValidator.hpp"

#include <array>

namespace {
void addError(ValidationResult& result, const std::string& message) {
    result.status = ValidationStatus::Error;
    result.messages.push_back(message);
}

void addWarning(ValidationResult& result, const std::string& message) {
    if (result.status != ValidationStatus::Error) {
        result.status = ValidationStatus::Warning;
    }
    result.messages.push_back(message);
}

bool robotCollidesWithObstacle(const MapData& mapData, const AMR& robot) {
    if (mapData.isObstacleAt(robot.getPosition())) {
        return true;
    }

    for (const auto& corner : robot.getCorners()) {
        if (mapData.isObstacleAt(corner)) {
            return true;
        }
    }

    const float gridSize = mapData.getGridResolution();
    for (const auto& obstacle : mapData.getObstacles()) {
        const sf::Vector2f topLeft = mapData.getMapper().gridToWorldTopLeft(obstacle);
        const sf::Vector2f topRight(topLeft.x + gridSize, topLeft.y);
        const sf::Vector2f bottomRight(topLeft.x + gridSize, topLeft.y + gridSize);
        const sf::Vector2f bottomLeft(topLeft.x, topLeft.y + gridSize);
        const sf::Vector2f center = mapData.getMapper().gridToWorldCenter(obstacle);

        const std::array<sf::Vector2f, 5> samplePoints = {
            topLeft,
            topRight,
            bottomRight,
            bottomLeft,
            center
        };

        for (const auto& point : samplePoints) {
            if (robot.containsPoint(point)) {
                return true;
            }
        }
    }

    return false;
}

bool isWorkZoneTooSmallOrInvalid(const WorkZone& zone, float gridSize) {
    return zone.bounds.size.x <= 0.0f
        || zone.bounds.size.y <= 0.0f
        || zone.bounds.size.x < gridSize
        || zone.bounds.size.y < gridSize;
}
}

ValidationResult MapValidator::validate(const MapData& mapData, const AMR& robot) {
    ValidationResult result;

    const auto& startPose = mapData.getRobotStartPose();
    const auto& goalPose = mapData.getRobotGoalPose();

    if (!startPose.has_value()) {
        addError(result, "Start pose is not set.");
    } else {
        if (!mapData.containsWorldPoint(startPose->position)) {
            addError(result, "Start pose is outside the world boundary.");
        }
        if (mapData.isObstacleAt(startPose->position)) {
            addError(result, "Start pose is on an obstacle.");
        }
    }

    if (!goalPose.has_value()) {
        addError(result, "Goal pose is not set.");
    } else {
        if (!mapData.containsWorldPoint(goalPose->position)) {
            addError(result, "Goal pose is outside the world boundary.");
        }
        if (mapData.isObstacleAt(goalPose->position)) {
            addError(result, "Goal pose is on an obstacle.");
        }
    }

    if (startPose.has_value() && goalPose.has_value()) {
        if (mapData.getMapper().worldToGrid(startPose->position)
            == mapData.getMapper().worldToGrid(goalPose->position)) {
            addWarning(result, "Start pose and goal pose are in the same grid cell.");
        }
    }

    if (robotCollidesWithObstacle(mapData, robot)) {
        addWarning(result, "Robot is colliding with an obstacle.");
    }

    if (mapData.getObstacles().empty()) {
        addWarning(result, "Map has no obstacles.");
    }

    const auto& workZones = mapData.getWorkZones();
    for (std::size_t i = 0; i < workZones.size(); ++i) {
        if (isWorkZoneTooSmallOrInvalid(workZones[i], mapData.getGridResolution())) {
            addWarning(result, "Work zone " + std::to_string(i + 1) + " is too small or invalid.");
        }
    }

    return result;
}
