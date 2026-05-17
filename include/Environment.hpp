#pragma once
#include <SFML/Graphics.hpp>
#include <set>
#include <utility> // 為了使用 std::pair

class Environment {
public:
    // Constructor (建構子)：初始化時設定網格尺寸
    Environment(float gridSize);

    // 新增障礙物：傳入從滑鼠轉換過來的「真實物理座標」
    void addObstacle(sf::Vector2f worldPos);

    // 繪製環境：負責畫出無限網格與所有被放置的障礙物方塊
    void draw(sf::RenderWindow& window, const sf::View& simView);

    // 取得網格大小，供外部運算參考
    float getGridSize() const;

    // 新增：給定一個實體座標，檢查該座標所在的網格是否為障礙物
    bool isObstacleAt(sf::Vector2f worldPos) const;

private:
    float m_gridSize;

    // 資料結構：儲存含有障礙物的網格索引 (X 欄, Y 列)
    std::set<std::pair<int, int>> m_obstacles;
};