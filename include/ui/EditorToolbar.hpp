#pragma once

#include <SFML/Graphics.hpp>
#include <array>
#include <optional>

#include "editor/Environment.hpp"

class EditorToolbar {
public:
    void setBounds(const sf::FloatRect& bounds);
    const sf::FloatRect& getBounds() const;
    std::optional<EditorMode> hitTest(const sf::Vector2i& pixelPosition) const;
    sf::FloatRect getButtonBounds(EditorMode mode) const;
    void draw(
        sf::RenderWindow& window,
        const sf::Font& font,
        bool hasFont,
        EditorMode activeMode
    ) const;

private:
    struct Button {
        EditorMode mode;
        const char* label;
        sf::FloatRect bounds;
    };

    void updateButtons();

    sf::FloatRect m_bounds;
    std::array<Button, 7> m_buttons{{
        {EditorMode::Select, "Select [S]", {}},
        {EditorMode::PlaceObstacle, "Obstacle [O]", {}},
        {EditorMode::DeleteObstacle, "Erase [E]", {}},
        {EditorMode::SetStartPose, "Start [T]", {}},
        {EditorMode::SetGoalPose, "Goal [G]", {}},
        {EditorMode::DrawWorkZone, "Zone [Z]", {}},
        {EditorMode::PanView, "Pan [P]", {}}
    }};
};
