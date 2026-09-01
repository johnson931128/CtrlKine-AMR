#include "ui/ApplicationLayout.hpp"

#include <algorithm>

ApplicationLayout calculateApplicationLayout(const sf::Vector2u& windowSize) {
    constexpr float kToolbarHeight = 64.0f;
    constexpr float kPreferredInspectorWidth = 360.0f;
    constexpr float kMinimumSimulationWidth = 320.0f;

    const float width = static_cast<float>(windowSize.x);
    const float height = static_cast<float>(windowSize.y);
    const float toolbarHeight = std::min(kToolbarHeight, height);
    const float inspectorWidth = std::min(
        kPreferredInspectorWidth,
        std::max(0.0f, width - kMinimumSimulationWidth)
    );
    const float contentHeight = std::max(0.0f, height - toolbarHeight);
    const float simulationWidth = std::max(0.0f, width - inspectorWidth);

    return ApplicationLayout{
        sf::FloatRect(sf::Vector2f(0.0f, 0.0f), sf::Vector2f(width, toolbarHeight)),
        sf::FloatRect(
            sf::Vector2f(0.0f, toolbarHeight),
            sf::Vector2f(simulationWidth, contentHeight)
        ),
        sf::FloatRect(
            sf::Vector2f(simulationWidth, toolbarHeight),
            sf::Vector2f(inspectorWidth, contentHeight)
        )
    };
}
