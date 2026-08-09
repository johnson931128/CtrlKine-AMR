#include "Simulator.hpp"

#include <array>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

namespace {
constexpr unsigned int kWindowWidth = 1280;
constexpr unsigned int kWindowHeight = 800;
constexpr float kToolbarHeight = 64.0f;
constexpr float kInspectorWidth = 260.0f;
constexpr float kToolbarButtonWidth = 42.0f;
constexpr float kToolbarButtonHeight = 30.0f;
constexpr float kToolbarButtonGap = 8.0f;
constexpr float kToolbarButtonStartX = 16.0f;
constexpr float kToolbarButtonY = 16.0f;
constexpr float kHeadingStepRadians = 0.17453292f;
constexpr float kInspectorPadding = 16.0f;
constexpr float kInspectorSectionGap = 18.0f;
constexpr float kInspectorTitleGap = 10.0f;
constexpr float kInspectorScrollStep = 48.0f;
const sf::Vector2f kDefaultRobotPosition(400.0f, 400.0f);
const sf::Color kPathColor(30, 144, 255, 220);

std::string toPathExecutionLabel(PathExecutionState state) {
    switch (state) {
    case PathExecutionState::Following:
        return "following";
    case PathExecutionState::Completed:
        return "completed";
    case PathExecutionState::NotFollowing:
    default:
        return "not following";
    }
}

bool loadUiFont(sf::Font& font) {
    const std::array<const char*, 4> fontPaths = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibri.ttf",
        "C:/Windows/Fonts/consola.ttf"
    };

    for (const char* path : fontPaths) {
        if (font.openFromFile(path)) {
            return true;
        }
    }

    return false;
}

std::string toValidationLabel(ValidationStatus status) {
    switch (status) {
    case ValidationStatus::Valid:
        return "Valid";
    case ValidationStatus::Warning:
        return "Warning";
    case ValidationStatus::Error:
    default:
        return "Error";
    }
}

sf::Color getValidationColor(ValidationStatus status) {
    switch (status) {
    case ValidationStatus::Valid:
        return sf::Color(46, 139, 87);
    case ValidationStatus::Warning:
        return sf::Color(184, 134, 11);
    case ValidationStatus::Error:
    default:
        return sf::Color(178, 34, 34);
    }
}

std::vector<std::string> splitWrappedLine(
    const std::string& text,
    const sf::Font& font,
    unsigned int characterSize,
    float maxWidth
) {
    std::vector<std::string> lines;
    if (text.empty()) {
        lines.push_back("");
        return lines;
    }

    sf::Text measure(font, "", characterSize);
    std::istringstream words(text);
    std::string word;
    std::string currentLine;

    while (words >> word) {
        const std::string candidate = currentLine.empty() ? word : currentLine + " " + word;
        measure.setString(candidate);
        if (!currentLine.empty() && measure.getLocalBounds().size.x > maxWidth) {
            lines.push_back(currentLine);
            currentLine = word;
            continue;
        }
        currentLine = candidate;
    }

    if (!currentLine.empty()) {
        lines.push_back(currentLine);
    }

    if (lines.empty()) {
        lines.push_back(text);
    }

    return lines;
}

std::vector<std::string> wrapTextLines(
    const std::string& text,
    const sf::Font& font,
    unsigned int characterSize,
    float maxWidth
) {
    std::vector<std::string> lines;
    std::istringstream input(text);
    std::string rawLine;

    while (std::getline(input, rawLine)) {
        const std::vector<std::string> wrapped = splitWrappedLine(rawLine, font, characterSize, maxWidth);
        lines.insert(lines.end(), wrapped.begin(), wrapped.end());
    }

    if (lines.empty()) {
        lines.push_back("");
    }

    return lines;
}
}

Simulator::Simulator()
    : m_window(sf::VideoMode({kWindowWidth, kWindowHeight}), "AMR Physics Simulator & Environment Editor"),
      m_hasUiFont(false),
      m_amrConfig{100.0f, 60.0f, 30.0f, 10.0f, 70.0f, 80.0f, sf::Color(100, 150, 250), sf::Color(50, 50, 50)},
      m_amr(m_amrConfig, kDefaultRobotPosition),
      m_env(50.0f),
      m_isPanning(false),
      m_lastPanPixel({0, 0}),
      m_simViewportRect(sf::Vector2f(0.0f, kToolbarHeight), sf::Vector2f(kWindowWidth - kInspectorWidth, kWindowHeight - kToolbarHeight)),
      m_defaultRobotPosition(kDefaultRobotPosition),
      m_inspectorScrollOffset(0.0f),
      m_inspectorContentHeight(0.0f),
      m_mapFilename("saved_map.txt"),
      m_statusMessage("Ready"),
      m_selectedObject(SelectedObject::none()) {
    loadConfig("config.txt");
    m_hasUiFont = loadUiFont(m_uiFont);
    m_amr = AMR(m_amrConfig, m_defaultRobotPosition);

    m_uiView = m_window.getDefaultView();
    m_simView = sf::View(
        sf::Vector2f({400.0f, 400.0f}),
        sf::Vector2f({m_simViewportRect.size.x, m_simViewportRect.size.y})
    );
    updateWindowLayout();

    m_pathVertices.setPrimitiveType(sf::PrimitiveType::LineStrip);
    PathResult initialPathResult;
    initialPathResult.message = "Path planning has not run yet.";
    m_pathExecution.install(std::move(initialPathResult));
    updateValidationResult();
}

void Simulator::updateWindowLayout() {
    const sf::Vector2u windowSize = m_window.getSize();
    const float windowWidth = static_cast<float>(windowSize.x);
    const float windowHeight = static_cast<float>(windowSize.y);

    const float previousDefaultWidth = m_defaultSimView.getSize().x > 0.0f ? m_defaultSimView.getSize().x : m_simView.getSize().x;
    const float zoomFactor = previousDefaultWidth > 0.0f ? (m_simView.getSize().x / previousDefaultWidth) : 1.0f;
    const sf::Vector2f currentCenter = m_simView.getCenter();

    m_uiView = sf::View(
        sf::Vector2f(windowWidth * 0.5f, windowHeight * 0.5f),
        sf::Vector2f(windowWidth, windowHeight)
    );

    m_simViewportRect = sf::FloatRect(
        sf::Vector2f(0.0f, kToolbarHeight),
        sf::Vector2f(windowWidth - kInspectorWidth, windowHeight - kToolbarHeight)
    );

    const sf::Vector2f defaultSimSize(m_simViewportRect.size.x, m_simViewportRect.size.y);
    m_defaultSimView = sf::View(sf::Vector2f(400.0f, 400.0f), defaultSimSize);
    m_simView.setCenter(currentCenter);
    m_simView.setSize(sf::Vector2f(defaultSimSize.x * zoomFactor, defaultSimSize.y * zoomFactor));
    m_simView.setViewport(sf::FloatRect(
        {m_simViewportRect.position.x / windowWidth, m_simViewportRect.position.y / windowHeight},
        {m_simViewportRect.size.x / windowWidth, m_simViewportRect.size.y / windowHeight}
    ));

    m_toolbarBg.setSize(sf::Vector2f(windowWidth, kToolbarHeight));
    m_toolbarBg.setPosition(sf::Vector2f(0.0f, 0.0f));
    m_toolbarBg.setFillColor(sf::Color(236, 236, 236));

    m_inspectorBg.setSize(sf::Vector2f(kInspectorWidth, windowHeight - kToolbarHeight));
    m_inspectorBg.setPosition(sf::Vector2f(windowWidth - kInspectorWidth, kToolbarHeight));
    m_inspectorBg.setFillColor(sf::Color(244, 244, 244));

    m_divider.setSize(sf::Vector2f(2.0f, windowHeight));
    m_divider.setPosition(sf::Vector2f(windowWidth - kInspectorWidth, 0.0f));
    m_divider.setFillColor(sf::Color(150, 150, 150));

    scrollInspector(0.0f);
}

void Simulator::scrollInspector(float delta) {
    const float visibleHeight = m_inspectorBg.getSize().y;
    const float maxScroll = std::max(0.0f, m_inspectorContentHeight - visibleHeight);
    m_inspectorScrollOffset = std::clamp(m_inspectorScrollOffset + delta, 0.0f, maxScroll);
}

void Simulator::loadConfig(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open " << filename << ". Using default configurations.\n";
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        float value;
        if (iss >> key >> value) {
            if (key == "bodyLength") m_amrConfig.bodyLength = value;
            else if (key == "bodyWidth") m_amrConfig.bodyWidth = value;
            else if (key == "wheelLength") m_amrConfig.wheelLength = value;
            else if (key == "wheelWidth") m_amrConfig.wheelWidth = value;
            else if (key == "trackWidth") m_amrConfig.trackWidth = value;
            else if (key == "wheelBase") m_amrConfig.wheelBase = value;
        }
    }
}

void Simulator::saveMap() {
    m_statusMessage = m_env.saveMapToFile(m_mapFilename)
        ? "Saved map to " + m_mapFilename
        : "Failed to save map";
}

void Simulator::loadMap() {
    if (m_env.loadMapFromFile(m_mapFilename)) {
        clearPathExecution();
        clearSelection();
        updateValidationResult();
        m_statusMessage = "Loaded map from " + m_mapFilename;
    } else {
        m_statusMessage = "Failed to load map";
    }
}

void Simulator::clearMap() {
    m_env.clearMap();
    clearPathExecution();
    clearSelection();
    updateValidationResult();
    m_statusMessage = "Cleared map";
}

void Simulator::resetView() {
    m_simView = m_defaultSimView;
    m_isPanning = false;
    updateCursorPreview();
    m_statusMessage = "Reset view";
}

void Simulator::resetRobotPose() {
    m_amr = AMR(m_amrConfig, m_defaultRobotPosition);
    clearPathExecution();
    updateValidationResult();
    m_statusMessage = "Reset robot pose";
}

void Simulator::updateValidationResult(bool updateStatusMessage) {
    m_validationResult = MapValidator::validate(m_env.getMapData(), m_amr);

    if (updateStatusMessage) {
        m_statusMessage = "Validation: " + toValidationLabel(m_validationResult.status);
    }
}

void Simulator::runPathPlanning() {
    PathResult result;

    if (m_validationResult.status == ValidationStatus::Error) {
        result.message = "Planning blocked: fix map validation errors first.";
        m_pathExecution.install(std::move(result));
        rebuildPathVisualization();
        m_statusMessage = "Planning blocked";
        return;
    }

    result = PathPlanner::plan(m_env.getMapData(), m_amr.getConservativeBodyRadius());
    m_pathExecution.install(std::move(result));
    rebuildPathVisualization();
    m_statusMessage = m_pathExecution.getResult().message;
}

void Simulator::clearPathExecution() {
    m_pathExecution.clear();
    m_pathVertices.clear();
}

void Simulator::rebuildPathVisualization() {
    m_pathVertices.clear();
    if (!m_pathExecution.hasExecutablePath()) {
        return;
    }

    const CoordinateMapper& mapper = m_env.getMapData().getMapper();
    for (const GridCoord& cell : m_pathExecution.getResult().path) {
        m_pathVertices.append(sf::Vertex{mapper.gridToWorldCenter(cell), kPathColor});
    }
}

void Simulator::handleEditorHotkeys(const sf::Event& event) {
    if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>()) {
        const bool isCtrlPressed =
            sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl)
            || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RControl);

        if (isCtrlPressed) {
            switch (keyPressed->code) {
            case sf::Keyboard::Key::N:
                clearMap();
                return;
            case sf::Keyboard::Key::Num0:
                resetView();
                return;
            case sf::Keyboard::Key::R:
                resetRobotPose();
                return;
            default:
                break;
            }
        }

        switch (keyPressed->code) {
        case sf::Keyboard::Key::Num1:
        case sf::Keyboard::Key::S:
            m_env.setEditorMode(EditorMode::Select);
            break;
        case sf::Keyboard::Key::Num2:
        case sf::Keyboard::Key::O:
            m_env.setEditorMode(EditorMode::PlaceObstacle);
            break;
        case sf::Keyboard::Key::Num3:
            m_env.setEditorMode(EditorMode::DeleteObstacle);
            break;
        case sf::Keyboard::Key::Q:
            if (rotateSelectedHeading(-kHeadingStepRadians)) {
                m_statusMessage = "Rotated selected pose";
            }
            break;
        case sf::Keyboard::Key::E:
            if (rotateSelectedHeading(kHeadingStepRadians)) {
                m_statusMessage = "Rotated selected pose";
            } else {
                m_env.setEditorMode(EditorMode::DeleteObstacle);
            }
            break;
        case sf::Keyboard::Key::Num4:
        case sf::Keyboard::Key::T:
            m_env.setEditorMode(EditorMode::SetStartPose);
            break;
        case sf::Keyboard::Key::Num5:
        case sf::Keyboard::Key::G:
            m_env.setEditorMode(EditorMode::SetGoalPose);
            break;
        case sf::Keyboard::Key::Num6:
        case sf::Keyboard::Key::Z:
            m_env.setEditorMode(EditorMode::DrawWorkZone);
            break;
        case sf::Keyboard::Key::Num7:
        case sf::Keyboard::Key::P:
            m_env.setEditorMode(EditorMode::PanView);
            break;
        case sf::Keyboard::Key::V:
            updateValidationResult(true);
            break;
        case sf::Keyboard::Key::Enter:
            runPathPlanning();
            break;
        case sf::Keyboard::Key::Escape:
            m_env.cancelActiveTool();
            m_isPanning = false;
            if (m_env.getEditorMode() == EditorMode::DrawWorkZone) {
                m_env.setEditorMode(EditorMode::Select);
            }
            if (!m_selectedObject.isNone()) {
                clearSelection();
            }
            break;
        case sf::Keyboard::Key::Delete:
            if (deleteSelectedObject()) {
                m_statusMessage = "Deleted selected object";
            }
            break;
        case sf::Keyboard::Key::F5:
            saveMap();
            break;
        case sf::Keyboard::Key::F9:
            loadMap();
            break;
        default:
            break;
        }
    }
}

bool Simulator::handleToolbarClick(const sf::Vector2i& pixelPos) {
    const std::array<EditorMode, 7> modes = {{
        EditorMode::Select,
        EditorMode::PlaceObstacle,
        EditorMode::DeleteObstacle,
        EditorMode::SetStartPose,
        EditorMode::SetGoalPose,
        EditorMode::DrawWorkZone,
        EditorMode::PanView
    }};

    for (std::size_t i = 0; i < modes.size(); ++i) {
        const float x = kToolbarButtonStartX + static_cast<float>(i) * (kToolbarButtonWidth + kToolbarButtonGap);
        const sf::FloatRect buttonRect(
            sf::Vector2f(x, kToolbarButtonY),
            sf::Vector2f(kToolbarButtonWidth, kToolbarButtonHeight)
        );

        if (buttonRect.contains(sf::Vector2f(static_cast<float>(pixelPos.x), static_cast<float>(pixelPos.y)))) {
            m_env.setEditorMode(modes[i]);
            m_statusMessage = "Switched mode";
            return true;
        }
    }

    return false;
}

void Simulator::selectObjectAt(const sf::Vector2f& worldPos) {
    SelectedObject selectedObject = m_env.hitTest(worldPos);
    if (selectedObject.isNone() && m_amr.containsPoint(worldPos)) {
        selectedObject = SelectedObject::robot();
    }

    m_selectedObject = selectedObject;
    syncSelectionToEnvironment();
    m_statusMessage = m_selectedObject.isNone() ? "Selection cleared" : "Selected object";
}

void Simulator::clearSelection() {
    m_selectedObject = SelectedObject::none();
    syncSelectionToEnvironment();
}

bool Simulator::deleteSelectedObject() {
    if (m_selectedObject.isNone() || m_selectedObject.type == SelectedObjectType::Robot) {
        return false;
    }

    const bool deleted = m_env.deleteSelectedObject(m_selectedObject);
    if (deleted) {
        clearPathExecution();
        clearSelection();
        updateValidationResult();
    }
    return deleted;
}

bool Simulator::rotateSelectedHeading(float deltaRadians) {
    if (m_env.getEditorMode() != EditorMode::Select) {
        return false;
    }

    if (m_selectedObject.type != SelectedObjectType::StartPose
        && m_selectedObject.type != SelectedObjectType::GoalPose) {
        return false;
    }

    return m_env.rotateSelectedHeading(m_selectedObject, deltaRadians);
}

void Simulator::syncSelectionToEnvironment() {
    m_env.setSelectedObject(m_selectedObject);
}

void Simulator::updateCursorPreview() {
    const sf::Vector2i mousePixel = sf::Mouse::getPosition(m_window);
    const bool insideSimX = mousePixel.x >= static_cast<int>(m_simViewportRect.position.x)
        && mousePixel.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x);
    const bool insideSimY = mousePixel.y >= static_cast<int>(m_simViewportRect.position.y)
        && mousePixel.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y);

    if (insideSimX && insideSimY) {
        m_hoverWorldPos = m_window.mapPixelToCoords(mousePixel, m_simView);
        m_env.setCursorWorldPosition(m_hoverWorldPos);
        return;
    }

    m_hoverWorldPos.reset();
    m_env.setCursorWorldPosition(std::nullopt);
}

void Simulator::run() {
    while (m_window.isOpen()) {
        processEvents();
        const float dt = m_clock.restart().asSeconds();
        update(dt);
        render();
    }
}

void Simulator::processEvents() {
    while (const std::optional<sf::Event> event = m_window.pollEvent()) {
        handleEditorHotkeys(*event);

        if (event->is<sf::Event::Closed>()) {
            m_window.close();
        }

        if (const auto* resized = event->getIf<sf::Event::Resized>()) {
            m_window.setView(m_window.getDefaultView());
            updateWindowLayout();
            (void)resized;
        }

        if (const auto* scroll = event->getIf<sf::Event::MouseWheelScrolled>()) {
            const sf::Vector2i mousePos = sf::Mouse::getPosition(m_window);
            const bool insideInspector = mousePos.x >= static_cast<int>(m_inspectorBg.getPosition().x)
                && mousePos.x < static_cast<int>(m_inspectorBg.getPosition().x + m_inspectorBg.getSize().x)
                && mousePos.y >= static_cast<int>(m_inspectorBg.getPosition().y)
                && mousePos.y < static_cast<int>(m_inspectorBg.getPosition().y + m_inspectorBg.getSize().y);

            if (insideInspector) {
                scrollInspector(scroll->delta > 0 ? -kInspectorScrollStep : kInspectorScrollStep);
            } else if (mousePos.x >= static_cast<int>(m_simViewportRect.position.x)
                && mousePos.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x)
                && mousePos.y >= static_cast<int>(m_simViewportRect.position.y)
                && mousePos.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y)) {
                if (scroll->delta > 0) m_simView.zoom(0.9f);
                else if (scroll->delta < 0) m_simView.zoom(1.1f);
            }
        }

        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left
                && mouseBtn->position.y >= 0
                && mouseBtn->position.y < static_cast<int>(kToolbarHeight)
                && handleToolbarClick(mouseBtn->position)) {
                continue;
            }

            const bool insideSimArea = mouseBtn->position.x >= static_cast<int>(m_simViewportRect.position.x)
                && mouseBtn->position.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x)
                && mouseBtn->position.y >= static_cast<int>(m_simViewportRect.position.y)
                && mouseBtn->position.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y);
            if (mouseBtn->button == sf::Mouse::Button::Left && insideSimArea) {
                const sf::Vector2f worldPos = m_window.mapPixelToCoords(mouseBtn->position, m_simView);

                if (m_env.getEditorMode() == EditorMode::PanView) {
                    m_isPanning = true;
                    m_lastPanPixel = mouseBtn->position;
                } else if (m_env.getEditorMode() == EditorMode::Select) {
                    selectObjectAt(worldPos);
                } else if (m_env.getEditorMode() == EditorMode::DeleteObstacle) {
                    SelectedObject deletedObject = SelectedObject::none();
                    if (m_env.deleteObjectAt(worldPos, &deletedObject)) {
                        clearPathExecution();
                        if (m_selectedObject.type == deletedObject.type
                            && m_selectedObject.obstacleCoord == deletedObject.obstacleCoord
                            && m_selectedObject.workZoneIndex == deletedObject.workZoneIndex) {
                            clearSelection();
                        } else {
                            syncSelectionToEnvironment();
                        }
                        updateValidationResult();
                        m_statusMessage = "Deleted object";
                    }
                } else {
                    if (!m_env.isInsideWorldBounds(worldPos)) {
                        continue;
                    }
                    clearPathExecution();
                    m_env.handleLeftMousePressed(worldPos);
                    updateValidationResult();
                }
            }
        }

        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonReleased>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left) {
                const bool insideSimArea = mouseBtn->position.x >= static_cast<int>(m_simViewportRect.position.x)
                    && mouseBtn->position.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x)
                    && mouseBtn->position.y >= static_cast<int>(m_simViewportRect.position.y)
                    && mouseBtn->position.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y);
                if (insideSimArea) {
                    const sf::Vector2f worldPos = m_window.mapPixelToCoords(mouseBtn->position, m_simView);
                    m_env.handleLeftMouseReleased(worldPos);
                    updateValidationResult();
                }
                m_isPanning = false;
            }
        }

        if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
            if (m_isPanning && m_env.getEditorMode() == EditorMode::PanView) {
                const sf::Vector2f previousWorld = m_window.mapPixelToCoords(m_lastPanPixel, m_simView);
                const sf::Vector2f currentWorld = m_window.mapPixelToCoords(mouseMoved->position, m_simView);
                m_simView.move(previousWorld - currentWorld);
                m_lastPanPixel = mouseMoved->position;
            }
        }
    }

    updateCursorPreview();
}

void Simulator::update(float dt) {
    const float maxSpeed = 150.0f;
    const float turnRate = 100.0f;
    const float maxAutomaticAngularSpeed = m_amrConfig.trackWidth > 0.0f
        ? (2.0f * turnRate) / m_amrConfig.trackWidth
        : 0.0f;

    AMR backupAmr = m_amr;
    const sf::Vector2f previousPosition = m_amr.getPosition();
    const float previousHeading = m_amr.getHeading();
    const bool wasFollowing = m_pathExecution.getState() == PathExecutionState::Following;
    bool reachedWaypoint = false;

    if (wasFollowing) {
        const std::optional<GridCoord> waypoint = m_pathExecution.getCurrentWaypoint();
        if (waypoint.has_value()) {
            const sf::Vector2f target = m_env.getMapData().getMapper().gridToWorldCenter(*waypoint);
            reachedWaypoint = m_amr.moveToward(
                target,
                dt,
                maxSpeed,
                maxAutomaticAngularSpeed,
                0.5f
            );
        }
    } else {
        float linearSpeed = 0.0f;
        float turnSpeed = 0.0f;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up)) linearSpeed = maxSpeed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down)) linearSpeed = -maxSpeed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) turnSpeed = -turnRate;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) turnSpeed = turnRate;

        const float vL = linearSpeed + turnSpeed;
        const float vR = linearSpeed - turnSpeed;
        m_amr.update(dt, vL, vR);
    }

    const std::vector<sf::Vector2f> corners = m_amr.getCorners();

    bool collision = false;
    for (const auto& corner : corners) {
        if (!m_env.isInsideWorldBounds(corner) || m_env.isObstacleAt(corner)) {
            collision = true;
            break;
        }
    }

    if (collision) {
        m_amr = backupAmr;
    } else if (wasFollowing && reachedWaypoint) {
        m_pathExecution.advanceWaypoint();
        if (m_pathExecution.getState() == PathExecutionState::Completed) {
            m_statusMessage = "Path following completed.";
        }
    }

    const sf::Vector2f currentPosition = m_amr.getPosition();
    if (currentPosition.x != previousPosition.x
        || currentPosition.y != previousPosition.y
        || m_amr.getHeading() != previousHeading) {
        updateValidationResult();
    }
}

void Simulator::render() {
    m_window.clear(sf::Color(245, 245, 245));

    m_window.setView(m_simView);
    m_env.draw(m_window, m_simView);
    drawActivePath();
    m_amr.draw(m_window);

    if (m_selectedObject.type == SelectedObjectType::Robot) {
        const std::vector<sf::Vector2f> robotCorners = m_amr.getCorners();
        sf::VertexArray outline(sf::PrimitiveType::LineStrip, 5);
        for (std::size_t i = 0; i < 4; ++i) {
            outline[i] = sf::Vertex{robotCorners[i], sf::Color(255, 165, 0)};
        }
        outline[4] = sf::Vertex{robotCorners[0], sf::Color(255, 165, 0)};
        m_window.draw(outline);
    }

    m_window.setView(m_uiView);
    drawToolbar();
    drawInspector();

    m_window.display();
}

void Simulator::drawActivePath() {
    if (!m_pathExecution.hasExecutablePath()) {
        return;
    }

    if (m_pathVertices.getVertexCount() > 1) {
        m_window.draw(m_pathVertices);
    }

    sf::CircleShape waypointMarker(4.0f);
    waypointMarker.setOrigin(sf::Vector2f(4.0f, 4.0f));
    waypointMarker.setFillColor(kPathColor);

    const CoordinateMapper& mapper = m_env.getMapData().getMapper();
    for (const GridCoord& cell : m_pathExecution.getExecutionWaypoints()) {
        waypointMarker.setPosition(mapper.gridToWorldCenter(cell));
        m_window.draw(waypointMarker);
    }
}

void Simulator::drawToolbar() {
    m_window.draw(m_toolbarBg);

    if (!m_hasUiFont) {
        return;
    }

    const std::array<std::pair<EditorMode, std::string>, 7> tools = {{
        {EditorMode::Select, "S"},
        {EditorMode::PlaceObstacle, "O"},
        {EditorMode::DeleteObstacle, "E"},
        {EditorMode::SetStartPose, "T"},
        {EditorMode::SetGoalPose, "G"},
        {EditorMode::DrawWorkZone, "Z"},
        {EditorMode::PanView, "P"}
    }};

    float x = kToolbarButtonStartX;
    for (std::size_t i = 0; i < tools.size(); ++i) {
        const auto& [mode, label] = tools[i];
        sf::RectangleShape buttonBg(sf::Vector2f(kToolbarButtonWidth, kToolbarButtonHeight));
        buttonBg.setPosition(sf::Vector2f(x, kToolbarButtonY));
        buttonBg.setFillColor(
            mode == m_env.getEditorMode() ? sf::Color(70, 130, 180, 40) : sf::Color(255, 255, 255, 0)
        );
        buttonBg.setOutlineThickness(1.0f);
        buttonBg.setOutlineColor(
            mode == m_env.getEditorMode() ? sf::Color(70, 130, 180) : sf::Color(195, 195, 195)
        );
        m_window.draw(buttonBg);

        sf::Text toolText(m_uiFont, label, 16);
        toolText.setFillColor(mode == m_env.getEditorMode() ? sf::Color(25, 70, 120) : sf::Color(60, 60, 60));
        toolText.setPosition(sf::Vector2f(x + 13.0f, 22.0f));
        m_window.draw(toolText);
        x += kToolbarButtonWidth + kToolbarButtonGap;
    }
}

void Simulator::drawInspector() {
    m_window.draw(m_inspectorBg);
    m_window.draw(m_divider);

    if (!m_hasUiFont) {
        return;
    }

    const sf::Vector2u windowSize = m_window.getSize();
    const float windowWidth = static_cast<float>(windowSize.x);
    const float windowHeight = static_cast<float>(windowSize.y);
    const float contentWidth = kInspectorWidth - (kInspectorPadding * 2.0f);

    sf::View inspectorView(
        sf::FloatRect(
            sf::Vector2f(0.0f, 0.0f),
            sf::Vector2f(kInspectorWidth, m_inspectorBg.getSize().y)
        )
    );
    inspectorView.setViewport(sf::FloatRect(
        {m_inspectorBg.getPosition().x / windowWidth, m_inspectorBg.getPosition().y / windowHeight},
        {m_inspectorBg.getSize().x / windowWidth, m_inspectorBg.getSize().y / windowHeight}
    ));

    m_window.setView(inspectorView);

    float y = kInspectorPadding - m_inspectorScrollOffset;

    auto drawSection = [&](const std::string& heading, const std::string& body, const sf::Color& color) {
        sf::Text headingText(m_uiFont, heading, 19);
        headingText.setFillColor(sf::Color(45, 45, 45));
        headingText.setPosition(sf::Vector2f(kInspectorPadding, y));
        m_window.draw(headingText);
        y += 28.0f;

        const std::vector<std::string> wrappedLines = wrapTextLines(body, m_uiFont, 16, contentWidth);
        for (const auto& line : wrappedLines) {
            sf::Text bodyText(m_uiFont, line, 16);
            bodyText.setFillColor(color);
            bodyText.setPosition(sf::Vector2f(kInspectorPadding, y));
            m_window.draw(bodyText);
            y += 22.0f;
        }

        y += kInspectorSectionGap;
    };

    sf::Text title(m_uiFont, "Inspector", 24);
    title.setFillColor(sf::Color(40, 40, 40));
    title.setPosition(sf::Vector2f(kInspectorPadding, y));
    m_window.draw(title);
    y += 42.0f;

    std::ostringstream cursorInfo;
    if (m_hoverWorldPos.has_value()) {
        const GridCoord hoveredGrid = m_env.getMapData().getMapper().worldToGrid(*m_hoverWorldPos);
        cursorInfo << "World: (" << static_cast<int>(m_hoverWorldPos->x) << ", "
                   << static_cast<int>(m_hoverWorldPos->y) << ")\n"
                   << "Grid: (" << hoveredGrid.col << ", " << hoveredGrid.row << ")";
    } else {
        cursorInfo << "World: -\nGrid: -";
    }
    drawSection("Cursor", cursorInfo.str(), sf::Color(75, 75, 75));

    const MapData& map = m_env.getMapData();

    std::ostringstream mapInfo;
    mapInfo << "Grid: " << static_cast<int>(map.getGridResolution()) << "\n"
            << "Obstacles: " << map.getObstacles().size() << "\n"
            << "Work Zones: " << map.getWorkZones().size() << "\n"
            << "Start Pose: " << (map.getRobotStartPose().has_value() ? "set" : "unset") << "\n"
            << "Goal Pose: " << (map.getRobotGoalPose().has_value() ? "set" : "unset");
    drawSection("Map Stats", mapInfo.str(), sf::Color(75, 75, 75));

    std::ostringstream validationInfo;
    validationInfo << "Status: " << toValidationLabel(m_validationResult.status);
    if (m_validationResult.messages.empty()) {
        validationInfo << "\nMap is ready.";
    } else {
        for (const auto& message : m_validationResult.messages) {
            validationInfo << "\n- " << message;
        }
    }
    drawSection("Map Validation", validationInfo.str(), getValidationColor(m_validationResult.status));

    const PathResult& pathResult = m_pathExecution.getResult();
    std::ostringstream planningInfo;
    planningInfo << "Success: " << (pathResult.success ? "yes" : "no") << "\n"
                 << "Nodes: " << pathResult.nodesExpanded << "\n"
                 << "Length: " << static_cast<int>(pathResult.pathLength) << "\n"
                 << "Grid cells: " << pathResult.path.size() << "\n"
                 << "Waypoints: " << m_pathExecution.getExecutionWaypoints().size() << "\n"
                 << "Execution: " << toPathExecutionLabel(m_pathExecution.getState()) << "\n"
                 << "Message: " << pathResult.message;
    drawSection("Path Planning", planningInfo.str(), sf::Color(75, 75, 75));

    std::ostringstream selectedInfo;
    switch (m_selectedObject.type) {
    case SelectedObjectType::Obstacle:
        if (m_selectedObject.obstacleCoord.has_value()) {
            const sf::Vector2f worldTopLeft = map.getMapper().gridToWorldTopLeft(*m_selectedObject.obstacleCoord);
            selectedInfo << "Type: Obstacle\n"
                         << "Grid: (" << m_selectedObject.obstacleCoord->col << ", "
                         << m_selectedObject.obstacleCoord->row << ")\n"
                         << "World: (" << static_cast<int>(worldTopLeft.x) << ", "
                         << static_cast<int>(worldTopLeft.y) << ")";
        }
        break;
    case SelectedObjectType::WorkZone:
        if (m_selectedObject.workZoneIndex.has_value()
            && *m_selectedObject.workZoneIndex < map.getWorkZones().size()) {
            const WorkZone& zone = map.getWorkZones()[*m_selectedObject.workZoneIndex];
            selectedInfo << "Type: Work Zone\n"
                         << "Position: (" << static_cast<int>(zone.bounds.position.x) << ", "
                         << static_cast<int>(zone.bounds.position.y) << ")\n"
                         << "Size: (" << static_cast<int>(zone.bounds.size.x) << ", "
                         << static_cast<int>(zone.bounds.size.y) << ")";
        }
        break;
    case SelectedObjectType::StartPose:
        if (map.getRobotStartPose().has_value()) {
            selectedInfo << "Type: Start Pose\n"
                         << "Position: (" << static_cast<int>(map.getRobotStartPose()->position.x) << ", "
                         << static_cast<int>(map.getRobotStartPose()->position.y) << ")\n"
                         << "Heading: " << static_cast<int>(map.getRobotStartPose()->heading * 57.2958f) << " deg";
        }
        break;
    case SelectedObjectType::GoalPose:
        if (map.getRobotGoalPose().has_value()) {
            selectedInfo << "Type: Goal Pose\n"
                         << "Position: (" << static_cast<int>(map.getRobotGoalPose()->position.x) << ", "
                         << static_cast<int>(map.getRobotGoalPose()->position.y) << ")\n"
                         << "Heading: " << static_cast<int>(map.getRobotGoalPose()->heading * 57.2958f) << " deg";
        }
        break;
    case SelectedObjectType::Robot:
        selectedInfo << "Type: Robot\n"
                     << "Position: (" << static_cast<int>(m_amr.getPosition().x) << ", "
                     << static_cast<int>(m_amr.getPosition().y) << ")\n"
                     << "Heading: " << static_cast<int>(m_amr.getHeading() * 57.2958f) << " deg";
        break;
    case SelectedObjectType::None:
    default:
        selectedInfo << "Type: None";
        break;
    }
    drawSection("Selected Object", selectedInfo.str(), sf::Color(75, 75, 75));

    const sf::Vector2f robotPos = m_amr.getPosition();
    std::ostringstream robotInfo;
    robotInfo << "Position: (" << static_cast<int>(robotPos.x) << ", "
              << static_cast<int>(robotPos.y) << ")\n"
              << "Heading: " << static_cast<int>(m_amr.getHeading() * 57.2958f) << " deg\n"
              << "Mode: "
              << (m_pathExecution.getState() == PathExecutionState::NotFollowing
                  ? "manual"
                  : toPathExecutionLabel(m_pathExecution.getState()));
    drawSection("Robot State", robotInfo.str(), sf::Color(75, 75, 75));

    std::ostringstream controlsInfo;
    controlsInfo << "Enter Plan Path\n"
                 << "V Validate Map\n"
                 << "F5 Save\n"
                 << "F9 Load\n"
                 << "Ctrl+N Clear Map\n"
                 << "Ctrl+0 Reset View\n"
                 << "Ctrl+R Reset Robot";
    drawSection("Controls", controlsInfo.str(), sf::Color(70, 70, 70));

    m_inspectorContentHeight = std::max(m_inspectorBg.getSize().y, y + m_inspectorScrollOffset);
    scrollInspector(0.0f);

    m_window.setView(m_uiView);
}
