#include "EditorToolbar.hpp"

#include <algorithm>

namespace {
bool containsPixel(const sf::FloatRect& bounds, const sf::Vector2i& pixel) {
    return bounds.contains(sf::Vector2f(
        static_cast<float>(pixel.x), static_cast<float>(pixel.y)
    ));
}
}

void EditorToolbar::setBounds(const sf::FloatRect& bounds) {
    m_bounds = bounds;
    updateButtons();
}

const sf::FloatRect& EditorToolbar::getBounds() const {
    return m_bounds;
}

std::optional<EditorMode> EditorToolbar::hitTest(const sf::Vector2i& pixelPosition) const {
    for (const Button& button : m_buttons) {
        if (containsPixel(button.bounds, pixelPosition)) {
            return button.mode;
        }
    }
    return std::nullopt;
}

sf::FloatRect EditorToolbar::getButtonBounds(EditorMode mode) const {
    for (const Button& button : m_buttons) {
        if (button.mode == mode) {
            return button.bounds;
        }
    }
    return {};
}

void EditorToolbar::draw(
    sf::RenderWindow& window,
    const sf::Font& font,
    bool hasFont,
    EditorMode activeMode
) const {
    sf::RectangleShape background(m_bounds.size);
    background.setPosition(m_bounds.position);
    background.setFillColor(sf::Color(236, 239, 243));
    window.draw(background);

    if (!hasFont) {
        return;
    }

    for (const Button& button : m_buttons) {
        const bool active = button.mode == activeMode;
        sf::RectangleShape buttonBackground(button.bounds.size);
        buttonBackground.setPosition(button.bounds.position);
        buttonBackground.setFillColor(
            active ? sf::Color(70, 130, 180, 45) : sf::Color(255, 255, 255, 190)
        );
        buttonBackground.setOutlineThickness(active ? 2.0f : 1.0f);
        buttonBackground.setOutlineColor(
            active ? sf::Color(55, 115, 170) : sf::Color(190, 197, 205)
        );
        window.draw(buttonBackground);

        sf::Text label(font, button.label, 14);
        label.setFillColor(active ? sf::Color(25, 70, 120) : sf::Color(58, 63, 69));
        const sf::FloatRect textBounds = label.getLocalBounds();
        label.setPosition(sf::Vector2f(
            button.bounds.position.x + (button.bounds.size.x - textBounds.size.x) * 0.5f,
            button.bounds.position.y + 8.0f
        ));
        window.draw(label);
    }
}

void EditorToolbar::updateButtons() {
    constexpr float kGap = 8.0f;
    constexpr float kHorizontalPadding = 16.0f;
    constexpr float kPreferredWidth = 118.0f;
    constexpr float kHeight = 34.0f;

    const float availableWidth = std::max(
        0.0f,
        m_bounds.size.x - 2.0f * kHorizontalPadding
            - kGap * static_cast<float>(m_buttons.size() - 1)
    );
    const float buttonWidth = std::min(
        kPreferredWidth,
        availableWidth / static_cast<float>(m_buttons.size())
    );
    float x = m_bounds.position.x + kHorizontalPadding;
    const float y = m_bounds.position.y + std::max(0.0f, (m_bounds.size.y - kHeight) * 0.5f);
    for (Button& button : m_buttons) {
        button.bounds = sf::FloatRect(
            sf::Vector2f(x, y),
            sf::Vector2f(std::max(0.0f, buttonWidth), std::min(kHeight, m_bounds.size.y))
        );
        x += buttonWidth + kGap;
    }
}
