#pragma once

#include <SFML/Graphics.hpp>
#include <array>
#include <optional>
#include <string>

#include "AMR.hpp"
#include "LocalizationVisualization.hpp"
#include "MapValidator.hpp"
#include "PathExecution.hpp"
#include "SelectedObject.hpp"
#include "SlamVisualization.hpp"

enum class InspectorTab {
    Map,
    Navigation,
    Localization,
    Slam
};

struct InspectorData {
    const MapData& map;
    const AMR& amr;
    const ValidationResult& validation;
    const PathExecution& pathExecution;
    const SelectedObject& selectedObject;
    std::optional<sf::Vector2f> hoverWorldPosition;
    const LocalizationEstimate& localizationEstimate;
    const LocalizationStatistics& localizationStatistics;
    Pose2D odometryPose;
    bool odometryInitialized = false;
    Pose2D groundTruthPose;
    const LocalizationViewOptions& localizationView;
    std::size_t localizationHistorySize = 0;
    bool localizationDrivenNavigation = false;
    std::string planningStartSource;
    std::string statusMessage;
    std::string navigationStatusMessage;
    bool kidnapTestActive = false;
    const SlamUpdateResult& slamUpdate;
    SlamMapStatistics slamMapStatistics;
    const SlamViewOptions& slamView;
};

class InspectorPanel {
public:
    void setBounds(const sf::FloatRect& bounds);
    const sf::FloatRect& getBounds() const;
    sf::FloatRect getBodyBounds() const;
    sf::FloatRect getFooterBounds() const;
    sf::FloatRect getTabBounds(InspectorTab tab) const;
    bool contains(const sf::Vector2i& pixelPosition) const;
    bool handleClick(const sf::Vector2i& pixelPosition);

    InspectorTab getActiveTab() const;
    float getScrollOffset(InspectorTab tab) const;
    void setContentHeight(InspectorTab tab, float height);
    void scroll(float delta);

    void draw(
        sf::RenderWindow& window,
        const sf::Font& font,
        bool hasFont,
        const InspectorData& data
    );

private:
    static std::size_t tabIndex(InspectorTab tab);
    void clampScroll(InspectorTab tab);

    sf::FloatRect m_bounds;
    InspectorTab m_activeTab = InspectorTab::Map;
    std::array<float, 4> m_scrollOffsets{};
    std::array<float, 4> m_contentHeights{};
};
