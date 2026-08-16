#pragma once

#include <SFML/Graphics.hpp>
#include <optional>

#include "MapData.hpp"
#include "SelectedObject.hpp"

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

    void handleLeftMousePressed(const sf::Vector2f& worldPos);
    void handleLeftMouseReleased(const sf::Vector2f& worldPos);
    void cancelActiveTool();
    void setEditorMode(EditorMode mode);
    EditorMode getEditorMode() const;
    void setCursorWorldPosition(const std::optional<sf::Vector2f>& worldPos);

    void draw(
        sf::RenderWindow& window,
        const sf::View& simView,
        const SelectedObject& selectedObject
    );
    float getGridSize() const;
    bool isObstacleAt(const sf::Vector2f& worldPos) const;
    bool isInsideWorldBounds(const sf::Vector2f& worldPos) const;
    const MapData& getMapData() const;
    bool saveMapToFile(const std::string& filename) const;
    bool loadMapFromFile(const std::string& filename);
    void clearMap();
    bool isDrawingWorkZone() const;
    SelectedObject hitTest(const sf::Vector2f& worldPos) const;
    bool deleteObjectAt(const sf::Vector2f& worldPos, SelectedObject* deletedObject = nullptr);
    bool deleteSelectedObject(const SelectedObject& selectedObject);
    bool rotateSelectedHeading(const SelectedObject& selectedObject, float deltaRadians);

private:
    void drawGrid(sf::RenderWindow& window, const sf::View& simView);
    void drawWorldBoundary(sf::RenderWindow& window);
    void drawObstacles(sf::RenderWindow& window);
    void drawWorkZones(sf::RenderWindow& window);
    void drawSelectionHighlight(
        sf::RenderWindow& window,
        const SelectedObject& selectedObject
    );
    void drawCursorPreview(sf::RenderWindow& window);
    void drawPoseMarker(sf::RenderWindow& window, const Pose2D& pose, const sf::Color& color);
    sf::FloatRect makeRectFromPoints(const sf::Vector2f& start, const sf::Vector2f& end) const;
    bool shouldDrawGridPreview() const;
    sf::Color getPreviewColor() const;
    bool isPointOnPoseMarker(const sf::Vector2f& worldPos, const Pose2D& pose) const;

    MapData m_map;
    EditorMode m_editorMode;
    std::optional<sf::Vector2f> m_pendingZoneStart;
    std::optional<sf::Vector2f> m_cursorWorldPos;
    bool m_isDrawingWorkZone;
};
