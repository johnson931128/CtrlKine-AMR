#include "Simulator.hpp"

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>

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
}

Simulator::Simulator()
    : m_window(sf::VideoMode({kWindowWidth, kWindowHeight}), "AMR Physics Simulator & Environment Editor"),
      m_hasUiFont(false),
      m_amrConfig{100.0f, 60.0f, 30.0f, 10.0f, 70.0f, 80.0f, sf::Color(100, 150, 250), sf::Color(50, 50, 50)},
      m_amr(m_amrConfig, sf::Vector2f({400.0f, 400.0f})),
      m_env(50.0f),
      m_isPanning(false),
      m_lastPanPixel({0, 0}),
      m_simViewportRect(sf::Vector2f(0.0f, kToolbarHeight), sf::Vector2f(kWindowWidth - kInspectorWidth, kWindowHeight - kToolbarHeight)),
      m_mapFilename("saved_map.txt"),
      m_statusMessage("Ready") {
    loadConfig("config.txt");
    m_hasUiFont = loadUiFont(m_uiFont);
    m_amr = AMR(m_amrConfig, sf::Vector2f({400.0f, 400.0f}));

    m_uiView = m_window.getDefaultView();
    m_simView = sf::View(
        sf::Vector2f({400.0f, 400.0f}),
        sf::Vector2f({m_simViewportRect.size.x, m_simViewportRect.size.y})
    );
    m_simView.setViewport(sf::FloatRect(
        {m_simViewportRect.position.x / static_cast<float>(kWindowWidth), m_simViewportRect.position.y / static_cast<float>(kWindowHeight)},
        {m_simViewportRect.size.x / static_cast<float>(kWindowWidth), m_simViewportRect.size.y / static_cast<float>(kWindowHeight)}
    ));

    m_toolbarBg.setSize(sf::Vector2f({static_cast<float>(kWindowWidth), kToolbarHeight}));
    m_toolbarBg.setPosition(sf::Vector2f({0.0f, 0.0f}));
    m_toolbarBg.setFillColor(sf::Color(236, 236, 236));

    m_inspectorBg.setSize(sf::Vector2f({kInspectorWidth, static_cast<float>(kWindowHeight) - kToolbarHeight}));
    m_inspectorBg.setPosition(sf::Vector2f({kWindowWidth - kInspectorWidth, kToolbarHeight}));
    m_inspectorBg.setFillColor(sf::Color(244, 244, 244));

    m_divider.setSize(sf::Vector2f({2.0f, static_cast<float>(kWindowHeight)}));
    m_divider.setPosition(sf::Vector2f({kWindowWidth - kInspectorWidth, 0.0f}));
    m_divider.setFillColor(sf::Color(150, 150, 150));
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
    m_statusMessage = m_env.loadMapFromFile(m_mapFilename)
        ? "Loaded map from " + m_mapFilename
        : "Failed to load map";
}

void Simulator::handleEditorHotkeys(const sf::Event& event) {
    if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>()) {
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
        case sf::Keyboard::Key::E:
            m_env.setEditorMode(EditorMode::DeleteObstacle);
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
        case sf::Keyboard::Key::Escape:
            m_env.cancelActiveTool();
            m_isPanning = false;
            if (m_env.getEditorMode() == EditorMode::DrawWorkZone) {
                m_env.setEditorMode(EditorMode::Select);
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

        if (const auto* scroll = event->getIf<sf::Event::MouseWheelScrolled>()) {
            const sf::Vector2i mousePos = sf::Mouse::getPosition(m_window);
            if (mousePos.x >= static_cast<int>(m_simViewportRect.position.x)
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
                } else {
                    m_env.handleLeftMousePressed(worldPos);
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
    float linearSpeed = 0.0f;
    float turnSpeed = 0.0f;
    const float maxSpeed = 150.0f;
    const float turnRate = 100.0f;

    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up)) linearSpeed = maxSpeed;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down)) linearSpeed = -maxSpeed;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) turnSpeed = -turnRate;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) turnSpeed = turnRate;

    const float vL = linearSpeed + turnSpeed;
    const float vR = linearSpeed - turnSpeed;

    AMR backupAmr = m_amr;
    m_amr.update(dt, vL, vR);

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
    }
}

void Simulator::render() {
    m_window.clear(sf::Color(245, 245, 245));

    m_window.setView(m_simView);
    m_env.draw(m_window, m_simView);
    m_amr.draw(m_window);

    m_window.setView(m_uiView);
    drawToolbar();
    drawInspector();

    m_window.display();
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

    const float panelX = static_cast<float>(kWindowWidth) - kInspectorWidth + 16.0f;
    float y = kToolbarHeight + 18.0f;

    sf::Text title(m_uiFont, "Inspector", 24);
    title.setFillColor(sf::Color(40, 40, 40));
    title.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(title);
    y += 42.0f;

    sf::Text section1(m_uiFont, "Cursor", 19);
    section1.setFillColor(sf::Color(45, 45, 45));
    section1.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(section1);
    y += 28.0f;

    std::ostringstream cursorInfo;
    if (m_hoverWorldPos.has_value()) {
        const GridCoord hoveredGrid = m_env.getMapData().getMapper().worldToGrid(*m_hoverWorldPos);
        cursorInfo << "World: (" << static_cast<int>(m_hoverWorldPos->x) << ", "
                   << static_cast<int>(m_hoverWorldPos->y) << ")\n"
                   << "Grid: (" << hoveredGrid.col << ", " << hoveredGrid.row << ")";
    } else {
        cursorInfo << "World: -\nGrid: -";
    }

    sf::Text cursorText(m_uiFont, cursorInfo.str(), 16);
    cursorText.setFillColor(sf::Color(75, 75, 75));
    cursorText.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(cursorText);
    y += 70.0f;

    sf::Text section2(m_uiFont, "Map Stats", 19);
    section2.setFillColor(sf::Color(45, 45, 45));
    section2.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(section2);
    y += 28.0f;

    const MapData& map = m_env.getMapData();
    std::ostringstream mapInfo;
    mapInfo << "Grid: " << static_cast<int>(map.getGridResolution()) << "\n"
            << "Obstacles: " << map.getObstacles().size() << "\n"
            << "Work Zones: " << map.getWorkZones().size() << "\n"
            << "Start Pose: " << (map.getRobotStartPose().has_value() ? "set" : "unset") << "\n"
            << "Goal Pose: " << (map.getRobotGoalPose().has_value() ? "set" : "unset");

    sf::Text mapText(m_uiFont, mapInfo.str(), 16);
    mapText.setFillColor(sf::Color(75, 75, 75));
    mapText.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(mapText);
    y += 120.0f;

    sf::Text section3(m_uiFont, "Robot State", 19);
    section3.setFillColor(sf::Color(45, 45, 45));
    section3.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(section3);
    y += 28.0f;

    const sf::Vector2f robotPos = m_amr.getPosition();
    std::ostringstream robotInfo;
    robotInfo << "Position: (" << static_cast<int>(robotPos.x) << ", "
              << static_cast<int>(robotPos.y) << ")\n"
              << "Heading: " << static_cast<int>(m_amr.getHeading() * 57.2958f) << " deg\n"
              << "Mode: manual";

    sf::Text robotText(m_uiFont, robotInfo.str(), 16);
    robotText.setFillColor(sf::Color(75, 75, 75));
    robotText.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(robotText);
    y += 86.0f;

    if (m_env.getEditorMode() == EditorMode::DrawWorkZone) {
        sf::Text zoneHint(
            m_uiFont,
            m_env.isDrawingWorkZone()
                ? "Work Zone:\nRelease mouse to create\nEsc to cancel"
                : "Work Zone:\nPress and drag to create\nEsc to cancel",
            16
        );
        zoneHint.setFillColor(sf::Color(70, 70, 70));
        zoneHint.setPosition(sf::Vector2f(panelX, y));
        m_window.draw(zoneHint);
        y += 78.0f;
    }

    sf::Text ioHint(m_uiFont, "F5 Save\nF9 Load", 16);
    ioHint.setFillColor(sf::Color(70, 70, 70));
    ioHint.setPosition(sf::Vector2f(panelX, y));
    m_window.draw(ioHint);

    sf::Text statusTitle(m_uiFont, "Status", 19);
    statusTitle.setFillColor(sf::Color(45, 45, 45));
    statusTitle.setPosition(sf::Vector2f(panelX, y + 62.0f));
    m_window.draw(statusTitle);

    sf::Text statusText(m_uiFont, m_statusMessage, 16);
    statusText.setFillColor(sf::Color(75, 75, 75));
    statusText.setPosition(sf::Vector2f(panelX, y + 92.0f));
    m_window.draw(statusText);
}
