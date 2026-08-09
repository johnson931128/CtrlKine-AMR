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

    return worldPos.x >= topLeft.x && worldPos.x < bottomRight.x
        && worldPos.y >= topLeft.y && worldPos.y < bottomRight.y;
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

    float gridResolution = 0.0f;
    sf::FloatRect worldBoundary;
    std::optional<Pose2D> startPose;
    std::optional<Pose2D> goalPose;
    std::vector<GridCoord> obstacles;
    std::vector<sf::FloatRect> workZones;
    std::size_t declaredObstacleCount = 0;
    std::size_t declaredWorkZoneCount = 0;

    bool hasGridResolution = false;
    bool hasWorldBoundary = false;
    bool hasStartPose = false;
    bool hasGoalPose = false;
    bool hasObstacleCount = false;
    bool hasWorkZoneCount = false;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        std::istringstream iss(line);
        std::string key;
        if (!(iss >> key)) {
            continue;
        }

        std::string extra;

        if (key == "grid_resolution") {
            float resolution = 0.0f;
            if (!(iss >> resolution) || (iss >> extra) || !(resolution > 0.0f)) {
                return false;
            }
            gridResolution = resolution;
            hasGridResolution = true;
        } else if (key == "world_boundary") {
            float x = 0.0f;
            float y = 0.0f;
            float width = 0.0f;
            float height = 0.0f;
            if (!(iss >> x >> y >> width >> height) || (iss >> extra)
                || !(width > 0.0f) || !(height > 0.0f)) {
                return false;
            }
            worldBoundary = sf::FloatRect(sf::Vector2f(x, y), sf::Vector2f(width, height));
            hasWorldBoundary = true;
        } else if (key == "start_pose") {
            std::string marker;
            if (!(iss >> marker)) {
                return false;
            }

            if (marker == "none") {
                if (iss >> extra) {
                    return false;
                }
                startPose.reset();
            } else {
                std::istringstream xStream(marker);
                float x = 0.0f;
                float y = 0.0f;
                float heading = 0.0f;
                if (!(xStream >> x) || (xStream >> extra)
                    || !(iss >> y >> heading) || (iss >> extra)) {
                    return false;
                }
                startPose = Pose2D{sf::Vector2f(x, y), heading};
            }
            hasStartPose = true;
        } else if (key == "goal_pose") {
            std::string marker;
            if (!(iss >> marker)) {
                return false;
            }

            if (marker == "none") {
                if (iss >> extra) {
                    return false;
                }
                goalPose.reset();
            } else {
                std::istringstream xStream(marker);
                float x = 0.0f;
                float y = 0.0f;
                float heading = 0.0f;
                if (!(xStream >> x) || (xStream >> extra)
                    || !(iss >> y >> heading) || (iss >> extra)) {
                    return false;
                }
                goalPose = Pose2D{sf::Vector2f(x, y), heading};
            }
            hasGoalPose = true;
        } else if (key == "obstacle_count") {
            long long count = 0;
            if (!(iss >> count) || (iss >> extra) || count < 0) {
                return false;
            }
            declaredObstacleCount = static_cast<std::size_t>(count);
            hasObstacleCount = true;
        } else if (key == "work_zone_count") {
            long long count = 0;
            if (!(iss >> count) || (iss >> extra) || count < 0) {
                return false;
            }
            declaredWorkZoneCount = static_cast<std::size_t>(count);
            hasWorkZoneCount = true;
        } else if (key == "obstacle") {
            GridCoord coord;
            if (!(iss >> coord.col >> coord.row) || (iss >> extra)) {
                return false;
            }
            obstacles.push_back(coord);
        } else if (key == "work_zone") {
            float x = 0.0f;
            float y = 0.0f;
            float width = 0.0f;
            float height = 0.0f;
            if (!(iss >> x >> y >> width >> height) || (iss >> extra)
                || !(width > 0.0f) || !(height > 0.0f)) {
                return false;
            }
            workZones.emplace_back(sf::Vector2f(x, y), sf::Vector2f(width, height));
        } else {
            return false;
        }
    }

    if (file.bad() || !hasGridResolution || !hasWorldBoundary || !hasStartPose
        || !hasGoalPose || !hasObstacleCount || !hasWorkZoneCount
        || declaredObstacleCount != obstacles.size()
        || declaredWorkZoneCount != workZones.size()) {
        return false;
    }

    MapData loadedMap(gridResolution);
    loadedMap.setWorldBoundary(worldBoundary);

    for (const GridCoord& obstacle : obstacles) {
        if (!loadedMap.containsWorldPoint(loadedMap.getMapper().gridToWorldCenter(obstacle))) {
            return false;
        }
        loadedMap.addObstacle(obstacle);
    }

    for (const sf::FloatRect& workZone : workZones) {
        const std::size_t previousCount = loadedMap.getWorkZones().size();
        loadedMap.addWorkZone(workZone);
        if (loadedMap.getWorkZones().size() == previousCount) {
            return false;
        }
    }

    if (startPose.has_value()) {
        loadedMap.setRobotStartPose(*startPose);
        if (!loadedMap.getRobotStartPose().has_value()) {
            return false;
        }
    }
    if (goalPose.has_value()) {
        loadedMap.setRobotGoalPose(*goalPose);
        if (!loadedMap.getRobotGoalPose().has_value()) {
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

    m_obstacles.insert(m_mapper.worldToGrid(worldPos));
}

void MapData::addObstacle(const GridCoord& coord) {
    if (!containsWorldPoint(m_mapper.gridToWorldCenter(coord))) {
        return;
    }

    m_obstacles.insert(coord);
}

void MapData::removeObstacle(const sf::Vector2f& worldPos) {
    m_obstacles.erase(m_mapper.worldToGrid(worldPos));
}

void MapData::removeObstacle(const GridCoord& coord) {
    m_obstacles.erase(coord);
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

bool MapData::removeWorkZone(std::size_t index) {
    if (index >= m_workZones.size()) {
        return false;
    }

    m_workZones.erase(m_workZones.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
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

void MapData::clearRobotStartPose() {
    m_robotStartPose.reset();
}

void MapData::clearRobotGoalPose() {
    m_robotGoalPose.reset();
}

const std::optional<Pose2D>& MapData::getRobotStartPose() const {
    return m_robotStartPose;
}

const std::optional<Pose2D>& MapData::getRobotGoalPose() const {
    return m_robotGoalPose;
}
