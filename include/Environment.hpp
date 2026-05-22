#pragma once

#include <SFML/Graphics.hpp>
#include <optional>

#include "MapData.hpp"

enum class EditorMode {
    Select,
    PlaceObstacle,
    DeleteObstacle,
    SetStartPose,
    SetGoalPose,
    DrawWorkZone,
    PanView
};

class Environment {
public:
    explicit Environment(float gridSize);

    void handleLeftClick(const sf::Vector2f& worldPos);
    void cancelActiveTool();
    void setEditorMode(EditorMode mode);
    EditorMode getEditorMode() const;
    void setCursorWorldPosition(const std::optional<sf::Vector2f>& worldPos);

    void draw(sf::RenderWindow& window, const sf::View& simView);
    float getGridSize() const;
    bool isObstacleAt(const sf::Vector2f& worldPos) const;
    bool isInsideWorldBounds(const sf::Vector2f& worldPos) const;
    const MapData& getMapData() const;

private:
    void drawGrid(sf::RenderWindow& window, const sf::View& simView);
    void drawWorldBoundary(sf::RenderWindow& window);
    void drawObstacles(sf::RenderWindow& window);
    void drawWorkZones(sf::RenderWindow& window);
    void drawCursorPreview(sf::RenderWindow& window);
    void drawPoseMarker(sf::RenderWindow& window, const Pose2D& pose, const sf::Color& color);
    sf::FloatRect makeRectFromPoints(const sf::Vector2f& start, const sf::Vector2f& end) const;
    bool shouldDrawGridPreview() const;
    sf::Color getPreviewColor() const;

    MapData m_map;
    EditorMode m_editorMode;
    std::optional<sf::Vector2f> m_pendingZoneStart;
    std::optional<sf::Vector2f> m_cursorWorldPos;
};
