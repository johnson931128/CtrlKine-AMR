#include "Simulator.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

Simulator::Simulator()
    : m_window(sf::VideoMode({1200u, 800u}), "AMR Physics Simulator & Environment Editor"),
      m_amrConfig{100.0f, 60.0f, 30.0f, 10.0f, 70.0f, 80.0f, sf::Color(100, 150, 250), sf::Color(50, 50, 50)},
      m_amr(m_amrConfig, sf::Vector2f({400.0f, 400.0f})),
      m_env(50.0f),
      m_isPanning(false),
      m_lastPanPixel({0, 0}) {
    loadConfig("config.txt");
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
    m_window.draw(m_toolbarBg);
    m_window.draw(m_divider);

    m_window.display();
}
