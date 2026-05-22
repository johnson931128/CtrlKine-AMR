#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>

struct GridCoord {
    int col = 0;
    int row = 0;

    bool operator<(const GridCoord& other) const {
        if (col != other.col) {
            return col < other.col;
        }
        return row < other.row;
    }

    bool operator==(const GridCoord& other) const {
        return col == other.col && row == other.row;
    }
};

struct Pose2D {
    sf::Vector2f position{0.0f, 0.0f};
    float heading = 0.0f;
};

class CoordinateMapper {
public:
    explicit CoordinateMapper(float gridResolution = 50.0f)
        : m_gridResolution(gridResolution) {}

    float getGridResolution() const { return m_gridResolution; }
    void setGridResolution(float gridResolution) { m_gridResolution = gridResolution; }

    GridCoord worldToGrid(const sf::Vector2f& worldPos) const {
        return GridCoord{
            static_cast<int>(std::floor(worldPos.x / m_gridResolution)),
            static_cast<int>(std::floor(worldPos.y / m_gridResolution))
        };
    }

    sf::Vector2f gridToWorldTopLeft(const GridCoord& coord) const {
        return sf::Vector2f(
            coord.col * m_gridResolution,
            coord.row * m_gridResolution
        );
    }

    sf::Vector2f gridToWorldCenter(const GridCoord& coord) const {
        const sf::Vector2f topLeft = gridToWorldTopLeft(coord);
        return sf::Vector2f(
            topLeft.x + (m_gridResolution * 0.5f),
            topLeft.y + (m_gridResolution * 0.5f)
        );
    }

    sf::Vector2f snapWorldToGridCorner(const sf::Vector2f& worldPos) const {
        return sf::Vector2f(
            std::round(worldPos.x / m_gridResolution) * m_gridResolution,
            std::round(worldPos.y / m_gridResolution) * m_gridResolution
        );
    }

private:
    float m_gridResolution;
};
