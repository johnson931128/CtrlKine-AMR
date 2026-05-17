#include "Environment.hpp"
#include <cmath> // 引入 cmath 以使用 floor 與 ceil

// Constructor (建構子)：接收網格大小並儲存
Environment::Environment(float gridSize) : m_gridSize(gridSize) {}

// 新增障礙物核心邏輯
void Environment::addObstacle(sf::Vector2f worldPos) {
    // 座標映射 (Coordinate Mapping)：將真實物理座標轉換為網格索引 (Grid Index)
    // 透過除以網格大小並無條件捨去 (floor)，精準定位出該點落在哪一個方格內
    int col = static_cast<int>(std::floor(worldPos.x / m_gridSize));
    int row = static_cast<int>(std::floor(worldPos.y / m_gridSize));
    
    // 將算出的索引加入集合中
    // 註：std::set 的特性會自動過濾重複值，因此同一個網格被點擊多次也不會影響記憶體或繪圖效能
    m_obstacles.insert({col, row});
}

// 繪製環境：包含底層網格與上層障礙物
void Environment::draw(sf::RenderWindow& window, const sf::View& simView) {
    // --- 1. 繪製無限網格 (Infinite Grid) ---
    sf::Vector2f center = simView.getCenter();
    sf::Vector2f size = simView.getSize();

    float left = center.x - size.x / 2.0f;
    float right = center.x + size.x / 2.0f;
    float top = center.y - size.y / 2.0f;
    float bottom = center.y + size.y / 2.0f;

    int startCol = static_cast<int>(std::floor(left / m_gridSize));
    int endCol = static_cast<int>(std::ceil(right / m_gridSize));
    int startRow = static_cast<int>(std::floor(top / m_gridSize));
    int endRow = static_cast<int>(std::ceil(bottom / m_gridSize));

    // 使用 sf::VertexArray 高效繪製線條 (SFML 3.0.0 聚合初始化寫法)
    sf::VertexArray lines(sf::PrimitiveType::Lines);
    sf::Color gridColor(210, 210, 210);
    sf::Color axisColor(150, 150, 150);

    for (int col = startCol; col <= endCol; ++col) {
        float x = col * m_gridSize;
        sf::Color color = (col == 0) ? axisColor : gridColor;
        lines.append(sf::Vertex{sf::Vector2f(x, top), color});
        lines.append(sf::Vertex{sf::Vector2f(x, bottom), color});
    }

    for (int row = startRow; row <= endRow; ++row) {
        float y = row * m_gridSize;
        sf::Color color = (row == 0) ? axisColor : gridColor;
        lines.append(sf::Vertex{sf::Vector2f(left, y), color});
        lines.append(sf::Vertex{sf::Vector2f(right, y), color});
    }

    window.draw(lines);

    // --- 2. 繪製障礙物 (Obstacles) ---
    // 效能優化 (Performance Optimization)：
    // 只在迴圈外部宣告「一個」方塊形狀，而不是為每個障礙物都建立新的物件。
    sf::RectangleShape obstacleShape(sf::Vector2f(m_gridSize, m_gridSize));
    obstacleShape.setFillColor(sf::Color(100, 100, 100)); // 深灰色
    
    // 設定內縮的外框線 (Outline)，使得相鄰的方塊也能明顯看出網格分隔
    obstacleShape.setOutlineThickness(-1.0f);
    obstacleShape.setOutlineColor(sf::Color(50, 50, 50));

    // 遍歷 (Iterate) 集合中的每一個網格索引，更新座標並繪製
    for (const auto& index : m_obstacles) {
        // 將索引反向轉換為實體螢幕座標
        float x = index.first * m_gridSize;
        float y = index.second * m_gridSize;
        obstacleShape.setPosition(sf::Vector2f(x, y));
        window.draw(obstacleShape);
    }
}


// 檢查特定物理座標是否落在障礙物網格內
bool Environment::isObstacleAt(sf::Vector2f worldPos) const {
    // 1. 將實體座標轉換為網格索引 (與 addObstacle 的邏輯完全相同)
    int col = static_cast<int>(std::floor(worldPos.x / m_gridSize));
    int row = static_cast<int>(std::floor(worldPos.y / m_gridSize));

    // 2. 在集合中尋找該索引。如果找到了 (不等於 end)，代表撞到障礙物了
    return m_obstacles.find({col, row}) != m_obstacles.end();
}

// 取得網格大小
float Environment::getGridSize() const {
    return m_gridSize;
}