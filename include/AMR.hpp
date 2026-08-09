#pragma once

#include <SFML/Graphics.hpp>
#include <cmath>
#include <vector>

struct AMRConfig {
    float bodyWidth;
    float bodyLength;
    float wheelWidth;
    float wheelLength;
    float trackWidth;
    float wheelBase;
    sf::Color bodyColor;
    sf::Color wheelColor;
};

class AMR {
public:
    AMR(const AMRConfig& config, sf::Vector2f startPos);

    void update(float dt, float vL, float vR);
    bool moveToward(const sf::Vector2f& target, float dt, float maxSpeed, float arrivalTolerance = 0.5f);
    void draw(sf::RenderWindow& window);

    sf::Vector2f getPosition() const { return m_position; }
    float getHeading() const { return m_heading; }
    std::vector<sf::Vector2f> getCorners() const;
    bool containsPoint(const sf::Vector2f& worldPos) const;

private:
    void syncShapes();

    AMRConfig m_config;
    sf::Vector2f m_position;
    float m_heading;
    sf::RectangleShape m_bodyShape;
    sf::RectangleShape m_wheels[4];
};
