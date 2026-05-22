#include "MapData.hpp"

namespace {
bool isValidRect(const sf::FloatRect& rect) {
    return rect.size.x > 0.0f && rect.size.y > 0.0f;
}
}

MapData::MapData(float gridResolution)
    : m_mapper(gridResolution),
      m_worldBoundary(sf::Vector2f(-1000.0f, -1000.0f), sf::Vector2f(2000.0f, 2000.0f)) {}

float MapData::getGridResolution() const {
    return m_mapper.getGridResolution();
}

const CoordinateMapper& MapData::getMapper() const {
    return m_mapper;
}

const sf::FloatRect& MapData::getWorldBoundary() const {
    return m_worldBoundary;
}

void MapData::setWorldBoundary(const sf::FloatRect& boundary) {
    m_worldBoundary = boundary;
}

bool MapData::containsWorldPoint(const sf::Vector2f& worldPos) const {
    const sf::Vector2f topLeft = m_worldBoundary.position;
    const sf::Vector2f bottomRight(
        m_worldBoundary.position.x + m_worldBoundary.size.x,
        m_worldBoundary.position.y + m_worldBoundary.size.y
    );

    return worldPos.x >= topLeft.x && worldPos.x <= bottomRight.x
        && worldPos.y >= topLeft.y && worldPos.y <= bottomRight.y;
}

void MapData::addObstacle(const sf::Vector2f& worldPos) {
    if (!containsWorldPoint(worldPos)) {
        return;
    }

    addObstacle(m_mapper.worldToGrid(worldPos));
}

void MapData::addObstacle(const GridCoord& coord) {
    m_obstacles.insert(coord);
}

void MapData::removeObstacle(const sf::Vector2f& worldPos) {
    m_obstacles.erase(m_mapper.worldToGrid(worldPos));
}

bool MapData::isObstacleAt(const sf::Vector2f& worldPos) const {
    return isObstacleAt(m_mapper.worldToGrid(worldPos));
}

bool MapData::isObstacleAt(const GridCoord& coord) const {
    return m_obstacles.find(coord) != m_obstacles.end();
}

const std::set<GridCoord>& MapData::getObstacles() const {
    return m_obstacles;
}

void MapData::addWorkZone(const sf::FloatRect& bounds) {
    if (!isValidRect(bounds)) {
        return;
    }

    const sf::Vector2f topLeft = bounds.position;
    const sf::Vector2f bottomRight(
        bounds.position.x + bounds.size.x,
        bounds.position.y + bounds.size.y
    );

    if (!containsWorldPoint(topLeft) || !containsWorldPoint(bottomRight)) {
        return;
    }

    m_workZones.push_back(WorkZone{bounds});
}

const std::vector<WorkZone>& MapData::getWorkZones() const {
    return m_workZones;
}

void MapData::setRobotStartPose(const Pose2D& pose) {
    if (containsWorldPoint(pose.position)) {
        m_robotStartPose = pose;
    }
}

void MapData::setRobotGoalPose(const Pose2D& pose) {
    if (containsWorldPoint(pose.position)) {
        m_robotGoalPose = pose;
    }
}

const std::optional<Pose2D>& MapData::getRobotStartPose() const {
    return m_robotStartPose;
}

const std::optional<Pose2D>& MapData::getRobotGoalPose() const {
    return m_robotGoalPose;
}
