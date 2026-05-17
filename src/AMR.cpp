// AMR.cpp
#include "AMR.hpp"

AMR::AMR(const AMRConfig& config, sf::Vector2f startPos) 
    : m_config(config), m_position(startPos), m_heading(0.0f) {
    
    // 1. 設定車體 (Body) 的大小與顏色
    m_bodyShape.setSize(sf::Vector2f(m_config.bodyLength, m_config.bodyWidth));
    m_bodyShape.setFillColor(m_config.bodyColor);
    
    // 將 Origin (原點) 設定在車體幾何正中心，這對於後續的旋轉 (Rotation) 極為重要
    m_bodyShape.setOrigin(sf::Vector2f(m_config.bodyLength / 2.0f, m_config.bodyWidth / 2.0f));
    m_bodyShape.setPosition(m_position);

    // 2. 設定四個輪子 (Wheels) 的共用參數
    sf::Vector2f wheelSize(m_config.wheelLength, m_config.wheelWidth);
    sf::Vector2f wheelOrigin(m_config.wheelLength / 2.0f, m_config.wheelWidth / 2.0f);

    for (int i = 0; i < 4; ++i) {
        m_wheels[i].setSize(wheelSize);
        m_wheels[i].setOrigin(wheelOrigin);
        m_wheels[i].setFillColor(m_config.wheelColor);
    }

    // 3. 計算並設定 Relative Positions (相對位置)
    // 假設 X 軸正向為車頭朝向，Y 軸為車身左右 (SFML 座標系中 Y 軸是向下遞增的)
    float halfBase = m_config.wheelBase / 2.0f;
    float halfTrack = m_config.trackWidth / 2.0f;

    // 左前輪 (Front Left)
    m_wheels[0].setPosition(m_position + sf::Vector2f(halfBase, -halfTrack));
    // 右前輪 (Front Right)
    m_wheels[1].setPosition(m_position + sf::Vector2f(halfBase, halfTrack));
    // 左後輪 (Rear Left)
    m_wheels[2].setPosition(m_position + sf::Vector2f(-halfBase, -halfTrack));
    // 右後輪 (Rear Right)
    m_wheels[3].setPosition(m_position + sf::Vector2f(-halfBase, halfTrack));
}

void AMR::update(float dt, float vL, float vR) {
    // 1. Differential Drive (差速驅動) 核心公式
    // Linear Velocity (線速度)：左右兩輪速度的平均值
    float v = (vL + vR) / 2.0f;
    // Angular Velocity (角速度)：兩輪速度差除以輪距
    float omega = (vL - vR) / m_config.trackWidth;

    // 2. 更新 Heading angle (轉向角度，單位為 Radian 弧度)
    m_heading += omega * dt;

    // 3. 更新 Position (當前絕對座標)
    m_position.x += v * std::cos(m_heading) * dt;
    m_position.y += v * std::sin(m_heading) * dt;

    // 4. 同步車體的 SFML 圖形狀態
    m_bodyShape.setPosition(m_position);
    // SFML 3.0.0 專屬語法：直接傳入 sf::radians(弧度)
    m_bodyShape.setRotation(sf::radians(m_heading));

    // 5. 更新四個輪子的座標與角度
    float halfBase = m_config.wheelBase / 2.0f;
    float halfTrack = m_config.trackWidth / 2.0f;
    
    // Local Coordinates (局部座標)：定義四個輪子相對於車體正中心的初始位置
    sf::Vector2f localOffsets[4] = {
        sf::Vector2f(halfBase, -halfTrack),  // 左前
        sf::Vector2f(halfBase, halfTrack),   // 右前
        sf::Vector2f(-halfBase, -halfTrack), // 左後
        sf::Vector2f(-halfBase, halfTrack)   // 右後
    };

    for(int i = 0; i < 4; i++) {
        // Rotation Matrix (旋轉矩陣)：將局部座標旋轉到當前的車頭方向
        float rx = localOffsets[i].x * std::cos(m_heading) - localOffsets[i].y * std::sin(m_heading);
        float ry = localOffsets[i].x * std::sin(m_heading) + localOffsets[i].y * std::cos(m_heading);
        
        // 將旋轉後的相對偏移量加上車體絕對座標，即為輪子目前的真實座標
        m_wheels[i].setPosition(m_position + sf::Vector2f(rx, ry));
        m_wheels[i].setRotation(sf::radians(m_heading));
    }
}


// 取得車體四個頂點的實體座標 (考量當前的座標與旋轉角度)
std::vector<sf::Vector2f> AMR::getCorners() const {
    std::vector<sf::Vector2f> corners;
    float halfLength = m_config.bodyLength / 2.0f;
    float halfWidth = m_config.bodyWidth / 2.0f;

    // 定義車體在未旋轉狀態下，四個角相對於中心的偏移量
    // 假設 X 軸向右為正 (車身長)，Y 軸向下為正 (車身寬)
    sf::Vector2f localCorners[4] = {
        sf::Vector2f(halfLength, -halfWidth),  // 右上 (車頭右側)
        sf::Vector2f(halfLength, halfWidth),   // 右下 (車身右側)
        sf::Vector2f(-halfLength, halfWidth),  // 左下 (車尾左側)
        sf::Vector2f(-halfLength, -halfWidth)  // 左上 (車尾右側)
    };

    // 套用旋轉矩陣並加上當前座標
    for (int i = 0; i < 4; ++i) {
        float rx = localCorners[i].x * std::cos(m_heading) - localCorners[i].y * std::sin(m_heading);
        float ry = localCorners[i].x * std::sin(m_heading) + localCorners[i].y * std::cos(m_heading);
        corners.push_back(m_position + sf::Vector2f(rx, ry));
    }

    return corners;
}

void AMR::draw(sf::RenderWindow& window) {
    // 繪製順序 (Draw Order)：先畫輪子再畫車體，確保車身會覆蓋在輪軸上方，符合真實視覺
    for (int i = 0; i < 4; ++i) {
        window.draw(m_wheels[i]);
    }
    window.draw(m_bodyShape);
}