#pragma once

#include <SFML/Graphics.hpp>
#include <optional>
#include <string>

#include "AMR.hpp"
#include "Environment.hpp"

class Simulator {
public:
    Simulator();
    void run();

private:
    void processEvents();
    void update(float dt);
    void render();
    void loadConfig(const std::string& filename);
    void handleEditorHotkeys(const sf::Event& event);

    sf::RenderWindow m_window;
    sf::View m_uiView;
    sf::View m_simView;

    sf::RectangleShape m_toolbarBg;
    sf::RectangleShape m_divider;

    AMRConfig m_amrConfig;
    AMR m_amr;
    Environment m_env;

    sf::Clock m_clock;
    bool m_isPanning;
    sf::Vector2i m_lastPanPixel;
};
