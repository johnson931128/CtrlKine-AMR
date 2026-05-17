// AMR.hpp
#pragma once
#include <SFML/Graphics.hpp>
#include <cmath>

// AMRConfig (Configuration, 設定檔)：集中管理實體參數
struct AMRConfig {
    float bodyWidth;
    float bodyLength;
    float wheelWidth;
    float wheelLength;
    float trackWidth; // 輪距 (左右輪中心距離)
    float wheelBase;  // 軸距 (前後輪中心距離)
    sf::Color bodyColor;
    sf::Color wheelColor;
};

// AMR 類別：負責管理車體的狀態與繪製
class AMR {
public:
    // Constructor (建構子)：初始化時傳入設定檔與初始位置
    AMR(const AMRConfig& config, sf::Vector2f startPos);
    
    // 新增：狀態更新函式，負責計算運動學
    void update(float dt, float vL, float vR);

    // 將車體繪製到指定的 RenderWindow (渲染視窗)
    void draw(sf::RenderWindow& window);

    // 新增：取得車體當前的物理座標
    sf::Vector2f getPosition() const { return m_position; }
    
    // 新增：取得車體當前的朝向角度 (Radian)
    float getHeading() const { return m_heading; }

    // 新增：計算並回傳車體的四個頂點座標 (考慮旋轉)
    std::vector<sf::Vector2f> getCorners() const;

private:
    AMRConfig m_config;
    sf::Vector2f m_position; // 當前座標
    float m_heading;         // 轉向角度 (Heading angle)，供後續運動學使用

    // SFML 圖形元件
    sf::RectangleShape m_bodyShape;
    sf::RectangleShape m_wheels[4]; // 陣列儲存 4 個輪子：左前、右前、左後、右後
};