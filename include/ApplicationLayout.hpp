#pragma once

#include <SFML/Graphics.hpp>

struct ApplicationLayout {
    sf::FloatRect toolbar;
    sf::FloatRect simulationViewport;
    sf::FloatRect inspector;
};

ApplicationLayout calculateApplicationLayout(const sf::Vector2u& windowSize);
