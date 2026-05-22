#include "Simulator.hpp"

#include <array>
#include <fstream>
#include <iostream>
#include <sstream>

namespace {
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
    : m_window(sf::VideoMode({1200u, 800u}), "AMR Physics Simulator & Environment Editor"),
      m_hasUiFont(false),
      m_amrConfig{100.0f, 60.0f, 30.0f, 10.0f, 70.0f, 80.0f, sf::Color(100, 150, 250), sf::Color(50, 50, 50)},
      m_amr(m_amrConfig, sf::Vector2f({400.0f, 400.0f})),
      m_env(50.0f),
      m_isPanning(false),
      m_lastPanPixel({0, 0}) {
    loadConfig("config.txt");
    m_hasUiFont = loadUiFont(m_uiFont);
    m_amr = AMR(m_amrConfig, sf::Vector2f({400.0f, 400.0f}));

    m_uiView = m_window.getDefaultView();
    m_simView = sf::View(sf::Vector2f({400.0f, 400.0f}), sf::Vector2f({800.0f, 800.0f}));
    m_simView.setViewport(sf::FloatRect({0.0f, 0.0f}, {800.0f / 1200.0f, 1.0f}));

    m_toolbarBg.setSize(sf::Vector2f({400.0f, 800.0f}));
    m_toolbarBg.setPosition(sf::Vector2f({800.0f, 0.0f}));
    m_toolbarBg.setFillColor(sf::Color(220, 220, 220));

    m_divider.setSize(sf::Vector2f({5.0f, 800.0f}));
    m_divider.setPosition(sf::Vector2f({800.0f, 0.0f}));
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

void Simulator::handleEditorHotkeys(const sf::Event& event) {
    if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>()) {
        switch (keyPressed->code) {
        case sf::Keyboard::Key::Num1:
            m_env.setEditorMode(EditorMode::Select);
            break;
        case sf::Keyboard::Key::Num2:
            m_env.setEditorMode(EditorMode::PlaceObstacle);
            break;
        case sf::Keyboard::Key::Num3:
            m_env.setEditorMode(EditorMode::DeleteObstacle);
            break;
        case sf::Keyboard::Key::Num4:
            m_env.setEditorMode(EditorMode::SetStartPose);
            break;
        case sf::Keyboard::Key::Num5:
            m_env.setEditorMode(EditorMode::SetGoalPose);
            break;
        case sf::Keyboard::Key::Num6:
            m_env.setEditorMode(EditorMode::DrawWorkZone);
            break;
        case sf::Keyboard::Key::Num7:
            m_env.setEditorMode(EditorMode::PanView);
            break;
        case sf::Keyboard::Key::Escape:
            m_env.cancelActiveTool();
            m_isPanning = false;
            break;
        default:
            break;
        }
    }
}

std::string Simulator::getModeLabel(EditorMode mode) const {
    switch (mode) {
    case EditorMode::Select:
        return "Select";
    case EditorMode::PlaceObstacle:
        return "Add Obstacle";
    case EditorMode::DeleteObstacle:
        return "Delete Obstacle";
    case EditorMode::SetStartPose:
        return "Set Start";
    case EditorMode::SetGoalPose:
        return "Set Goal";
    case EditorMode::DrawWorkZone:
        return "Draw Work Zone";
    case EditorMode::PanView:
        return "Pan View";
    default:
        return "Unknown";
    }
}

void Simulator::updateCursorPreview() {
    const sf::Vector2i mousePixel = sf::Mouse::getPosition(m_window);
    if (mousePixel.x >= 0 && mousePixel.x < 800 && mousePixel.y >= 0 && mousePixel.y < 800) {
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
            if (sf::Mouse::getPosition(m_window).x < 800) {
                if (scroll->delta > 0) m_simView.zoom(0.9f);
                else if (scroll->delta < 0) m_simView.zoom(1.1f);
            }
        }

        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left && mouseBtn->position.x < 800) {
                const sf::Vector2f worldPos = m_window.mapPixelToCoords(mouseBtn->position, m_simView);

                if (m_env.getEditorMode() == EditorMode::PanView) {
                    m_isPanning = true;
                    m_lastPanPixel = mouseBtn->position;
                } else {
                    m_env.handleLeftClick(worldPos);
                }
            }
        }

        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonReleased>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left) {
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

    m_window.display();
}

void Simulator::drawToolbar() {
    m_window.draw(m_toolbarBg);
    m_window.draw(m_divider);

    if (!m_hasUiFont) {
        return;
    }

    sf::Text title(m_uiFont, "Editor Tools", 28);
    title.setFillColor(sf::Color(40, 40, 40));
    title.setPosition(sf::Vector2f(835.0f, 36.0f));
    m_window.draw(title);

    sf::Text currentMode(m_uiFont, "Current Mode: " + getModeLabel(m_env.getEditorMode()), 20);
    currentMode.setFillColor(sf::Color(30, 30, 30));
    currentMode.setPosition(sf::Vector2f(835.0f, 92.0f));
    m_window.draw(currentMode);

    const std::array<std::pair<EditorMode, std::string>, 7> tools = {{
        {EditorMode::Select, "[1] Select"},
        {EditorMode::PlaceObstacle, "[2] Add Obstacle"},
        {EditorMode::DeleteObstacle, "[3] Delete Obstacle"},
        {EditorMode::SetStartPose, "[4] Set Start"},
        {EditorMode::SetGoalPose, "[5] Set Goal"},
        {EditorMode::DrawWorkZone, "[6] Draw Work Zone"},
        {EditorMode::PanView, "[7] Pan View"}
    }};

    float y = 150.0f;
    for (const auto& [mode, label] : tools) {
        sf::RectangleShape buttonBg(sf::Vector2f(320.0f, 34.0f));
        buttonBg.setPosition(sf::Vector2f(830.0f, y - 4.0f));
        buttonBg.setFillColor(
            mode == m_env.getEditorMode() ? sf::Color(70, 130, 180, 40) : sf::Color(255, 255, 255, 0)
        );
        buttonBg.setOutlineThickness(mode == m_env.getEditorMode() ? 1.0f : 0.0f);
        buttonBg.setOutlineColor(sf::Color(70, 130, 180));
        m_window.draw(buttonBg);

        sf::Text toolText(m_uiFont, label, 18);
        toolText.setFillColor(mode == m_env.getEditorMode() ? sf::Color(25, 70, 120) : sf::Color(60, 60, 60));
        toolText.setPosition(sf::Vector2f(842.0f, y));
        m_window.draw(toolText);
        y += 40.0f;
    }

    sf::Text hint(m_uiFont, "Esc: cancel current tool", 17);
    hint.setFillColor(sf::Color(90, 90, 90));
    hint.setPosition(sf::Vector2f(835.0f, 455.0f));
    m_window.draw(hint);

    if (m_hoverWorldPos.has_value()) {
        const GridCoord hoveredGrid = m_env.getMapData().getMapper().worldToGrid(*m_hoverWorldPos);

        sf::Text cursorLabel(m_uiFont, "Cursor", 20);
        cursorLabel.setFillColor(sf::Color(40, 40, 40));
        cursorLabel.setPosition(sf::Vector2f(835.0f, 520.0f));
        m_window.draw(cursorLabel);

        std::ostringstream info;
        info << "World: (" << static_cast<int>(m_hoverWorldPos->x)
             << ", " << static_cast<int>(m_hoverWorldPos->y) << ")\n"
             << "Grid: (" << hoveredGrid.col << ", " << hoveredGrid.row << ")";

        sf::Text cursorInfo(m_uiFont, info.str(), 17);
        cursorInfo.setFillColor(sf::Color(70, 70, 70));
        cursorInfo.setPosition(sf::Vector2f(835.0f, 555.0f));
        m_window.draw(cursorInfo);
    }

    if (m_env.getEditorMode() == EditorMode::DrawWorkZone) {
        sf::Text zoneHint(m_uiFont, "Click once to start,\nmove mouse to preview,\nclick again to confirm.", 17);
        zoneHint.setFillColor(sf::Color(70, 70, 70));
        zoneHint.setPosition(sf::Vector2f(835.0f, 650.0f));
        m_window.draw(zoneHint);
    }
}
