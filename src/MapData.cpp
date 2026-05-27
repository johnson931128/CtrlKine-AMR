#include "MapData.hpp"

#include <fstream>
#include <sstream>

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

void MapData::setGridResolution(float gridResolution) {
    if (gridResolution > 0.0f) {
        m_mapper.setGridResolution(gridResolution);
    }
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

void MapData::clear() {
    m_obstacles.clear();
    m_workZones.clear();
    m_robotStartPose.reset();
    m_robotGoalPose.reset();
}

bool MapData::saveToFile(const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    file << "grid_resolution " << getGridResolution() << "\n";
    file << "world_boundary "
         << m_worldBoundary.position.x << " "
         << m_worldBoundary.position.y << " "
         << m_worldBoundary.size.x << " "
         << m_worldBoundary.size.y << "\n";

    if (m_robotStartPose.has_value()) {
        file << "start_pose "
             << m_robotStartPose->position.x << " "
             << m_robotStartPose->position.y << " "
             << m_robotStartPose->heading << "\n";
    } else {
        file << "start_pose none\n";
    }

    if (m_robotGoalPose.has_value()) {
        file << "goal_pose "
             << m_robotGoalPose->position.x << " "
             << m_robotGoalPose->position.y << " "
             << m_robotGoalPose->heading << "\n";
    } else {
        file << "goal_pose none\n";
    }

    file << "obstacle_count " << m_obstacles.size() << "\n";
    for (const auto& obstacle : m_obstacles) {
        file << "obstacle " << obstacle.col << " " << obstacle.row << "\n";
    }

    file << "work_zone_count " << m_workZones.size() << "\n";
    for (const auto& zone : m_workZones) {
        file << "work_zone "
             << zone.bounds.position.x << " "
             << zone.bounds.position.y << " "
             << zone.bounds.size.x << " "
             << zone.bounds.size.y << "\n";
    }

    return true;
}

bool MapData::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    MapData loadedMap(getGridResolution());
    loadedMap.setWorldBoundary(m_worldBoundary);

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        std::istringstream iss(line);
        std::string key;
        iss >> key;

        if (key == "grid_resolution") {
            float resolution = 0.0f;
            if (!(iss >> resolution) || resolution <= 0.0f) {
                return false;
            }
            loadedMap.setGridResolution(resolution);
        } else if (key == "world_boundary") {
            float x = 0.0f;
            float y = 0.0f;
            float width = 0.0f;
            float height = 0.0f;
            if (!(iss >> x >> y >> width >> height)) {
                return false;
            }
            loadedMap.setWorldBoundary(sf::FloatRect(sf::Vector2f(x, y), sf::Vector2f(width, height)));
        } else if (key == "start_pose") {
            std::string marker;
            if (!(iss >> marker)) {
                return false;
            }
            if (marker != "none") {
                float y = 0.0f;
                float heading = 0.0f;
                const float x = std::stof(marker);
                if (!(iss >> y >> heading)) {
                    return false;
                }
                loadedMap.setRobotStartPose(Pose2D{sf::Vector2f(x, y), heading});
            }
        } else if (key == "goal_pose") {
            std::string marker;
            if (!(iss >> marker)) {
                return false;
            }
            if (marker != "none") {
                float y = 0.0f;
                float heading = 0.0f;
                const float x = std::stof(marker);
                if (!(iss >> y >> heading)) {
                    return false;
                }
                loadedMap.setRobotGoalPose(Pose2D{sf::Vector2f(x, y), heading});
            }
        } else if (key == "obstacle_count" || key == "work_zone_count") {
            continue;
        } else if (key == "obstacle") {
            GridCoord coord;
            if (!(iss >> coord.col >> coord.row)) {
                return false;
            }
            loadedMap.addObstacle(coord);
        } else if (key == "work_zone") {
            float x = 0.0f;
            float y = 0.0f;
            float width = 0.0f;
            float height = 0.0f;
            if (!(iss >> x >> y >> width >> height)) {
                return false;
            }
            loadedMap.addWorkZone(sf::FloatRect(sf::Vector2f(x, y), sf::Vector2f(width, height)));
        } else {
            return false;
        }
    }

    *this = loadedMap;
    return true;
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
