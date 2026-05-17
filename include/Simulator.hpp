#pragma once
#include <SFML/Graphics.hpp>
#include <optional>
#include <string>
#include "AMR.hpp"
#include "Environment.hpp" // 新增：引入環境類別

class Simulator {
public:
    Simulator();
    void run();

private:
    void processEvents();
    void update(float dt);
    void render();
    void loadConfig(const std::string& filename);

    sf::RenderWindow m_window;
    sf::View m_uiView;
    sf::View m_simView;

    sf::RectangleShape m_toolbarBg;
    sf::RectangleShape m_divider;

    AMRConfig m_amrConfig;
    AMR m_amr;
    
    // 新增：宣告環境物件
    Environment m_env; 

    sf::Clock m_clock;
};