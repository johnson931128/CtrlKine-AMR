#pragma once

#include <SFML/Graphics.hpp>
#include <cstdint>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "core/CoordinateTypes.hpp"

struct WorkZone {
    sf::FloatRect bounds;
};

class MapData {
public:
    explicit MapData(float gridResolution = 50.0f);

    float getGridResolution() const;
    void setGridResolution(float gridResolution);
    const CoordinateMapper& getMapper() const;

    const sf::FloatRect& getWorldBoundary() const;
    std::uint64_t getGeometryRevision() const;
    void setWorldBoundary(const sf::FloatRect& boundary);
    bool containsWorldPoint(const sf::Vector2f& worldPos) const;
    void clear();
    bool saveToFile(const std::string& filename) const;
    bool loadFromFile(const std::string& filename);

    void addObstacle(const sf::Vector2f& worldPos);
    void addObstacle(const GridCoord& coord);
    void removeObstacle(const sf::Vector2f& worldPos);
    void removeObstacle(const GridCoord& coord);
    bool isObstacleAt(const sf::Vector2f& worldPos) const;
    bool isObstacleAt(const GridCoord& coord) const;
    const std::set<GridCoord>& getObstacles() const;

    void addWorkZone(const sf::FloatRect& bounds);
    bool removeWorkZone(std::size_t index);
    const std::vector<WorkZone>& getWorkZones() const;

    void setRobotStartPose(const Pose2D& pose);
    void setRobotGoalPose(const Pose2D& pose);
    void clearRobotStartPose();
    void clearRobotGoalPose();
    const std::optional<Pose2D>& getRobotStartPose() const;
    const std::optional<Pose2D>& getRobotGoalPose() const;

private:
    CoordinateMapper m_mapper;
    sf::FloatRect m_worldBoundary;
    std::set<GridCoord> m_obstacles;
    std::vector<WorkZone> m_workZones;
    std::optional<Pose2D> m_robotStartPose;
    std::optional<Pose2D> m_robotGoalPose;
    std::uint64_t m_geometryRevision = 0;
};
