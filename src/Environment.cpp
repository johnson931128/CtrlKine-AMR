#include "Environment.hpp"

#include <algorithm>
#include <cmath>

Environment::Environment(float gridSize)
    : m_map(gridSize), m_editorMode(EditorMode::PlaceObstacle), m_isDrawingWorkZone(false) {}

void Environment::handleLeftMousePressed(const sf::Vector2f& worldPos) {
    switch (m_editorMode) {
    case EditorMode::Select:
        break;
    case EditorMode::PlaceObstacle:
        m_map.addObstacle(worldPos);
        break;
    case EditorMode::DeleteObstacle:
        m_map.removeObstacle(worldPos);
        break;
    case EditorMode::SetStartPose:
        m_map.setRobotStartPose(Pose2D{worldPos, 0.0f});
        break;
    case EditorMode::SetGoalPose:
        m_map.setRobotGoalPose(Pose2D{worldPos, 0.0f});
        break;
    case EditorMode::DrawWorkZone:
        if (!m_map.containsWorldPoint(worldPos)) {
            return;
        }
        m_pendingZoneStart = m_map.getMapper().snapWorldToGridCorner(worldPos);
        m_isDrawingWorkZone = true;
        break;
    case EditorMode::PanView:
        break;
    }
}

void Environment::handleLeftMouseReleased(const sf::Vector2f& worldPos) {
    if (m_editorMode != EditorMode::DrawWorkZone || !m_isDrawingWorkZone || !m_pendingZoneStart.has_value()) {
        return;
    }

    const sf::Vector2f snappedEnd = m_map.getMapper().snapWorldToGridCorner(worldPos);
    const sf::FloatRect zoneRect = makeRectFromPoints(*m_pendingZoneStart, snappedEnd);
    m_map.addWorkZone(zoneRect);
    m_pendingZoneStart.reset();
    m_isDrawingWorkZone = false;
    m_editorMode = EditorMode::Select;
}

void Environment::cancelActiveTool() {
    m_pendingZoneStart.reset();
    m_isDrawingWorkZone = false;
}

void Environment::setEditorMode(EditorMode mode) {
    if (m_editorMode == mode) {
        return;
    }

    cancelActiveTool();
    m_editorMode = mode;
}

EditorMode Environment::getEditorMode() const {
    return m_editorMode;
}

void Environment::setCursorWorldPosition(const std::optional<sf::Vector2f>& worldPos) {
    m_cursorWorldPos = worldPos;
}

void Environment::draw(sf::RenderWindow& window, const sf::View& simView) {
    drawGrid(window, simView);
    drawWorldBoundary(window);
    drawWorkZones(window);
    drawObstacles(window);
    drawCursorPreview(window);

    if (m_map.getRobotStartPose().has_value()) {
        drawPoseMarker(window, *m_map.getRobotStartPose(), sf::Color(60, 179, 113));
    }

    if (m_map.getRobotGoalPose().has_value()) {
        drawPoseMarker(window, *m_map.getRobotGoalPose(), sf::Color(220, 20, 60));
    }
}

void Environment::drawGrid(sf::RenderWindow& window, const sf::View& simView) {
    const sf::Vector2f center = simView.getCenter();
    const sf::Vector2f size = simView.getSize();
    const float gridSize = getGridSize();

    const float left = center.x - size.x / 2.0f;
    const float right = center.x + size.x / 2.0f;
    const float top = center.y - size.y / 2.0f;
    const float bottom = center.y + size.y / 2.0f;

    const int startCol = static_cast<int>(std::floor(left / gridSize));
    const int endCol = static_cast<int>(std::ceil(right / gridSize));
    const int startRow = static_cast<int>(std::floor(top / gridSize));
    const int endRow = static_cast<int>(std::ceil(bottom / gridSize));

    sf::VertexArray lines(sf::PrimitiveType::Lines);
    const sf::Color gridColor(210, 210, 210);
    const sf::Color axisColor(150, 150, 150);

    for (int col = startCol; col <= endCol; ++col) {
        const float x = col * gridSize;
        const sf::Color color = (col == 0) ? axisColor : gridColor;
        lines.append(sf::Vertex{sf::Vector2f(x, top), color});
        lines.append(sf::Vertex{sf::Vector2f(x, bottom), color});
    }

    for (int row = startRow; row <= endRow; ++row) {
        const float y = row * gridSize;
        const sf::Color color = (row == 0) ? axisColor : gridColor;
        lines.append(sf::Vertex{sf::Vector2f(left, y), color});
        lines.append(sf::Vertex{sf::Vector2f(right, y), color});
    }

    window.draw(lines);
}

void Environment::drawWorldBoundary(sf::RenderWindow& window) {
    sf::RectangleShape boundaryShape(m_map.getWorldBoundary().size);
    boundaryShape.setPosition(m_map.getWorldBoundary().position);
    boundaryShape.setFillColor(sf::Color::Transparent);
    boundaryShape.setOutlineThickness(2.0f);
    boundaryShape.setOutlineColor(sf::Color(120, 120, 120));
    window.draw(boundaryShape);
}

void Environment::drawObstacles(sf::RenderWindow& window) {
    sf::RectangleShape obstacleShape(sf::Vector2f(getGridSize(), getGridSize()));
    obstacleShape.setFillColor(sf::Color(100, 100, 100));
    obstacleShape.setOutlineThickness(-1.0f);
    obstacleShape.setOutlineColor(sf::Color(50, 50, 50));

    for (const auto& coord : m_map.getObstacles()) {
        obstacleShape.setPosition(m_map.getMapper().gridToWorldTopLeft(coord));
        window.draw(obstacleShape);
    }
}

void Environment::drawWorkZones(sf::RenderWindow& window) {
    sf::RectangleShape zoneShape;
    zoneShape.setFillColor(sf::Color(100, 149, 237, 50));
    zoneShape.setOutlineThickness(2.0f);
    zoneShape.setOutlineColor(sf::Color(65, 105, 225));

    for (const auto& zone : m_map.getWorkZones()) {
        zoneShape.setPosition(zone.bounds.position);
        zoneShape.setSize(zone.bounds.size);
        window.draw(zoneShape);
    }

    if (m_pendingZoneStart.has_value()) {
        sf::CircleShape pendingMarker(6.0f);
        pendingMarker.setOrigin(sf::Vector2f(6.0f, 6.0f));
        pendingMarker.setPosition(*m_pendingZoneStart);
        pendingMarker.setFillColor(sf::Color(65, 105, 225));
        window.draw(pendingMarker);

        if (m_isDrawingWorkZone && m_cursorWorldPos.has_value() && m_map.containsWorldPoint(*m_cursorWorldPos)) {
            sf::RectangleShape previewZone;
            const sf::Vector2f snappedCursor = m_map.getMapper().snapWorldToGridCorner(*m_cursorWorldPos);
            const sf::FloatRect previewBounds = makeRectFromPoints(*m_pendingZoneStart, snappedCursor);
            previewZone.setPosition(previewBounds.position);
            previewZone.setSize(previewBounds.size);
            previewZone.setFillColor(sf::Color(65, 105, 225, 40));
            previewZone.setOutlineThickness(2.0f);
            previewZone.setOutlineColor(sf::Color(65, 105, 225, 190));
            window.draw(previewZone);
        }
    }
}

void Environment::drawCursorPreview(sf::RenderWindow& window) {
    if (!m_cursorWorldPos.has_value() || !m_map.containsWorldPoint(*m_cursorWorldPos)) {
        return;
    }

    if (shouldDrawGridPreview()) {
        sf::RectangleShape previewCell(sf::Vector2f(getGridSize(), getGridSize()));
        previewCell.setPosition(
            m_map.getMapper().gridToWorldTopLeft(m_map.getMapper().worldToGrid(*m_cursorWorldPos))
        );
        previewCell.setFillColor(sf::Color::Transparent);
        previewCell.setOutlineThickness(2.0f);
        previewCell.setOutlineColor(getPreviewColor());
        window.draw(previewCell);
        return;
    }

    if (m_editorMode == EditorMode::SetStartPose || m_editorMode == EditorMode::SetGoalPose) {
        drawPoseMarker(window, Pose2D{*m_cursorWorldPos, 0.0f}, getPreviewColor());
        return;
    }

    if (m_editorMode == EditorMode::DrawWorkZone) {
        sf::CircleShape cornerMarker(5.0f);
        cornerMarker.setOrigin(sf::Vector2f(5.0f, 5.0f));
        cornerMarker.setPosition(m_map.getMapper().snapWorldToGridCorner(*m_cursorWorldPos));
        cornerMarker.setFillColor(getPreviewColor());
        window.draw(cornerMarker);
    }
}

void Environment::drawPoseMarker(sf::RenderWindow& window, const Pose2D& pose, const sf::Color& color) {
    sf::CircleShape marker(10.0f, 24);
    marker.setOrigin(sf::Vector2f(10.0f, 10.0f));
    marker.setPosition(pose.position);
    marker.setFillColor(color);
    window.draw(marker);

    sf::VertexArray headingLine(sf::PrimitiveType::Lines, 2);
    headingLine[0] = sf::Vertex{pose.position, color};
    headingLine[1] = sf::Vertex{
        sf::Vector2f(
            pose.position.x + std::cos(pose.heading) * 18.0f,
            pose.position.y + std::sin(pose.heading) * 18.0f
        ),
        color
    };
    window.draw(headingLine);
}

sf::FloatRect Environment::makeRectFromPoints(const sf::Vector2f& start, const sf::Vector2f& end) const {
    const float left = std::min(start.x, end.x);
    const float top = std::min(start.y, end.y);
    const float width = std::abs(end.x - start.x);
    const float height = std::abs(end.y - start.y);
    return sf::FloatRect(sf::Vector2f(left, top), sf::Vector2f(width, height));
}

float Environment::getGridSize() const {
    return m_map.getGridResolution();
}

bool Environment::isObstacleAt(const sf::Vector2f& worldPos) const {
    return m_map.isObstacleAt(worldPos);
}

bool Environment::isInsideWorldBounds(const sf::Vector2f& worldPos) const {
    return m_map.containsWorldPoint(worldPos);
}

const MapData& Environment::getMapData() const {
    return m_map;
}

bool Environment::isDrawingWorkZone() const {
    return m_isDrawingWorkZone;
}

bool Environment::shouldDrawGridPreview() const {
    return m_editorMode == EditorMode::PlaceObstacle
        || m_editorMode == EditorMode::DeleteObstacle;
}

sf::Color Environment::getPreviewColor() const {
    switch (m_editorMode) {
    case EditorMode::PlaceObstacle:
        return sf::Color(46, 139, 87);
    case EditorMode::DeleteObstacle:
        return sf::Color(220, 20, 60);
    case EditorMode::SetStartPose:
        return sf::Color(60, 179, 113, 180);
    case EditorMode::SetGoalPose:
        return sf::Color(220, 20, 60, 180);
    case EditorMode::DrawWorkZone:
        return sf::Color(65, 105, 225, 180);
    case EditorMode::PanView:
        return sf::Color(255, 165, 0, 180);
    case EditorMode::Select:
    default:
        return sf::Color(80, 80, 80, 180);
    }
}
