#include "Simulator.hpp"
#include <fstream>
#include <iostream>
#include <sstream>

Simulator::Simulator()
    : m_window(sf::VideoMode({1200u, 800u}), "AMR Physics Simulator & Environment Editor"),
      m_amrConfig{100.0f, 60.0f, 30.0f, 10.0f, 70.0f, 80.0f, sf::Color(100, 150, 250), sf::Color(50, 50, 50)},
      m_amr(m_amrConfig, sf::Vector2f({400.0f, 400.0f})),
      m_env(50.0f) // 初始化環境，設定網格大小為 50 單位
{
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

void Simulator::run() {
    while (m_window.isOpen()) {
        processEvents();
        float dt = m_clock.restart().asSeconds();
        update(dt);
        render();
    }
}

void Simulator::processEvents() {
    while (const std::optional<sf::Event> event = m_window.pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            m_window.close();
        }
        
        // 滑鼠滾輪縮放
        if (const auto* scroll = event->getIf<sf::Event::MouseWheelScrolled>()) {
            if (sf::Mouse::getPosition(m_window).x < 800) {
                if (scroll->delta > 0) m_simView.zoom(0.9f);
                else if (scroll->delta < 0) m_simView.zoom(1.1f);
            }
        }
        
        // --- 新增：滑鼠點擊放置障礙物 ---
        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left) {
                // 確保只有點擊左側模擬區時才觸發
                if (mouseBtn->position.x < 800) {
                    // 關鍵技術：將作業系統給的「視窗像素座標」轉換為受縮放影響的「物理世界座標」
                    sf::Vector2f worldPos = m_window.mapPixelToCoords(
                        sf::Vector2i({mouseBtn->position.x, mouseBtn->position.y}), 
                        m_simView
                    );
                    
                    // 將真實物理座標交給環境去對齊網格並儲存
                    m_env.addObstacle(worldPos);
                }
            }
        }
    }
}

void Simulator::update(float dt) {
    float linearSpeed = 0.0f; 
    float turnSpeed = 0.0f;   
    const float MAX_SPEED = 150.0f;
    const float TURN_RATE = 100.0f; 

    // 1. 讀取鍵盤控制指令
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up)) linearSpeed = MAX_SPEED;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down)) linearSpeed = -MAX_SPEED;
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) turnSpeed = -TURN_RATE; 
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) turnSpeed = TURN_RATE;  

    float vL = linearSpeed + turnSpeed;
    float vR = linearSpeed - turnSpeed;

    // --- 碰撞偵測與狀態還原 (Collision Detection & Rollback) ---

    // 2. 備份當前的車體狀態 (包含位置、角度與圖形)
    AMR backupAmr = m_amr;

    // 3. 嘗試進行移動更新 (先走一步看看)
    m_amr.update(dt, vL, vR);

    // 4. 取得更新後的車體四個頂點
    std::vector<sf::Vector2f> corners = m_amr.getCorners();

    // 5. 檢查這四個角是否有任何一個踩進了障礙物網格
    bool collision = false;
    for (const auto& corner : corners) {
        if (m_env.isObstacleAt(corner)) {
            collision = true;
            break; // 只要有一個角撞到，就不需要檢查其他的了
        }
    }

    // 6. 如果發生碰撞，則將車體還原回移動前的狀態 (阻擋穿透)
    if (collision) {
        m_amr = backupAmr;
    }
}

void Simulator::render() {
    m_window.clear(sf::Color(245, 245, 245)); 
    
    // 切換到模擬區視角
    m_window.setView(m_simView);
    
    // 委託 Environment 類別畫出網格與障礙物方塊
    m_env.draw(m_window, m_simView); 
    m_amr.draw(m_window);
    
    // 切換到 UI 視角
    m_window.setView(m_uiView);
    m_window.draw(m_toolbarBg);
    m_window.draw(m_divider);
    
    m_window.display();
}