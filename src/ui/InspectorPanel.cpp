#include "ui/InspectorPanel.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

namespace {
constexpr float kPadding = 16.0f;
constexpr float kHeaderHeight = 94.0f;
constexpr float kFooterHeight = 112.0f;
constexpr float kMinimumBodyHeightWithFooter = 96.0f;
constexpr float kSectionGap = 16.0f;
constexpr float kScrollStep = 48.0f;

struct InspectorSection {
    std::string title;
    std::string body;
    sf::Color color = sf::Color(72, 76, 82);
};

float visibleFooterHeight(const sf::FloatRect& bounds) {
    return bounds.size.y >= kHeaderHeight + kFooterHeight + kMinimumBodyHeightWithFooter
        ? kFooterHeight
        : 0.0f;
}

bool containsPixel(const sf::FloatRect& bounds, const sf::Vector2i& pixel) {
    return bounds.contains(sf::Vector2f(
        static_cast<float>(pixel.x), static_cast<float>(pixel.y)
    ));
}

std::string localizationStateLabel(LocalizationState state) {
    switch (state) {
    case LocalizationState::Tracking: return "Tracking";
    case LocalizationState::Ambiguous: return "Ambiguous";
    case LocalizationState::Converged: return "Converged";
    case LocalizationState::Recovering: return "Recovering";
    case LocalizationState::Uninitialized:
    default: return "Uninitialized";
    }
}

std::string slamStateLabel(SlamState state) {
    switch (state) {
    case SlamState::Tracking: return "Tracking";
    case SlamState::Lost: return "Lost";
    case SlamState::Uninitialized:
    default: return "Uninitialized";
    }
}

std::string localizationSupportLabel(LocalizationSupport support) {
    switch (support) {
    case LocalizationSupport::Good: return "Good";
    case LocalizationSupport::Weak: return "Weak";
    case LocalizationSupport::Insufficient:
    default: return "Insufficient";
    }
}

std::string localizationInitializationLabel(LocalizationInitialization initialization) {
    switch (initialization) {
    case LocalizationInitialization::Local: return "Local";
    case LocalizationInitialization::Global: return "Global";
    case LocalizationInitialization::None:
    default: return "None";
    }
}

std::string pathExecutionLabel(PathExecutionState state) {
    switch (state) {
    case PathExecutionState::Following: return "following";
    case PathExecutionState::Completed: return "completed";
    case PathExecutionState::NotFollowing:
    default: return "not following";
    }
}

std::string validationLabel(ValidationStatus status) {
    switch (status) {
    case ValidationStatus::Valid: return "Valid";
    case ValidationStatus::Warning: return "Warning";
    case ValidationStatus::Error:
    default: return "Error";
    }
}

sf::Color validationColor(ValidationStatus status) {
    switch (status) {
    case ValidationStatus::Valid: return sf::Color(36, 128, 78);
    case ValidationStatus::Warning: return sf::Color(165, 116, 0);
    case ValidationStatus::Error:
    default: return sf::Color(178, 34, 34);
    }
}

std::vector<std::string> wrapText(
    const std::string& text,
    const sf::Font& font,
    unsigned int characterSize,
    float maximumWidth
) {
    std::vector<std::string> lines;
    std::istringstream paragraphs(text);
    std::string paragraph;
    sf::Text measure(font, "", characterSize);
    while (std::getline(paragraphs, paragraph)) {
        if (paragraph.empty()) {
            lines.emplace_back();
            continue;
        }
        std::istringstream words(paragraph);
        std::string word;
        std::string line;
        while (words >> word) {
            const std::string candidate = line.empty() ? word : line + " " + word;
            measure.setString(candidate);
            if (!line.empty() && measure.getLocalBounds().size.x > maximumWidth) {
                lines.push_back(line);
                line = word;
            } else {
                line = candidate;
            }
        }
        lines.push_back(line);
    }
    if (lines.empty()) {
        lines.emplace_back();
    }
    return lines;
}

std::string poseSummary(const std::optional<Pose2D>& pose) {
    if (!pose.has_value()) {
        return "Not set";
    }
    std::ostringstream output;
    output << std::fixed << std::setprecision(1)
           << "X: " << pose->position.x << "\n"
           << "Y: " << pose->position.y << "\n"
           << "Heading: " << pose->heading * 57.2957795 << " deg";
    return output.str();
}

std::string selectedObjectSummary(const InspectorData& data) {
    const SelectedObject& selected = data.selectedObject;
    std::ostringstream output;
    switch (selected.type) {
    case SelectedObjectType::Obstacle:
        if (selected.obstacleCoord.has_value()) {
            const sf::Vector2f world = data.map.getMapper().gridToWorldTopLeft(*selected.obstacleCoord);
            output << "Type: Obstacle\nGrid: (" << selected.obstacleCoord->col << ", "
                   << selected.obstacleCoord->row << ")\nWorld: ("
                   << static_cast<int>(world.x) << ", " << static_cast<int>(world.y) << ")";
        }
        break;
    case SelectedObjectType::WorkZone:
        if (selected.workZoneIndex.has_value()
            && *selected.workZoneIndex < data.map.getWorkZones().size()) {
            const WorkZone& zone = data.map.getWorkZones()[*selected.workZoneIndex];
            output << "Type: Work Zone\nPosition: ("
                   << static_cast<int>(zone.bounds.position.x) << ", "
                   << static_cast<int>(zone.bounds.position.y) << ")\nSize: ("
                   << static_cast<int>(zone.bounds.size.x) << ", "
                   << static_cast<int>(zone.bounds.size.y) << ")";
        }
        break;
    case SelectedObjectType::StartPose:
        output << "Type: Start Pose\n" << poseSummary(data.map.getRobotStartPose());
        break;
    case SelectedObjectType::GoalPose:
        output << "Type: Goal Pose\n" << poseSummary(data.map.getRobotGoalPose());
        break;
    case SelectedObjectType::Robot:
        output << "Type: Robot\nPosition: (" << static_cast<int>(data.amr.getPosition().x)
               << ", " << static_cast<int>(data.amr.getPosition().y) << ")\nHeading: "
               << static_cast<int>(data.amr.getHeading() * 57.2957795f) << " deg";
        break;
    case SelectedObjectType::None:
    default:
        output << "Type: None";
        break;
    }
    return output.str();
}

std::vector<InspectorSection> buildMapSections(const InspectorData& data) {
    std::vector<InspectorSection> sections;
    std::ostringstream cursor;
    if (data.hoverWorldPosition.has_value()) {
        const GridCoord grid = data.map.getMapper().worldToGrid(*data.hoverWorldPosition);
        cursor << "World: (" << static_cast<int>(data.hoverWorldPosition->x) << ", "
               << static_cast<int>(data.hoverWorldPosition->y) << ")\nGrid: ("
               << grid.col << ", " << grid.row << ")";
    } else {
        cursor << "World: -\nGrid: -";
    }
    sections.push_back({"Cursor", cursor.str()});

    std::ostringstream stats;
    stats << "Grid: " << static_cast<int>(data.map.getGridResolution()) << "\n"
          << "Obstacles: " << data.map.getObstacles().size() << "\n"
          << "Work Zones: " << data.map.getWorkZones().size();
    sections.push_back({"Map Stats", stats.str()});

    std::ostringstream validation;
    validation << "Status: " << validationLabel(data.validation.status);
    if (data.validation.messages.empty()) {
        validation << "\nMap is ready.";
    } else {
        for (const std::string& message : data.validation.messages) {
            validation << "\n- " << message;
        }
    }
    sections.push_back({"Map Validation", validation.str(), validationColor(data.validation.status)});
    sections.push_back({"Selected Object", selectedObjectSummary(data)});
    sections.push_back({"Start", poseSummary(data.map.getRobotStartPose())});
    sections.push_back({"Goal", poseSummary(data.map.getRobotGoalPose())});
    sections.push_back({
        "Map Controls",
        "V Validate  |  F5 Save  |  F9 Load\nCtrl+N Clear  |  Ctrl+0 Reset View"
    });
    sections.push_back({"Application Status", data.statusMessage});
    return sections;
}

std::vector<InspectorSection> buildNavigationSections(const InspectorData& data) {
    std::vector<InspectorSection> sections;
    const sf::Vector2f robotPosition = data.amr.getPosition();
    std::ostringstream robot;
    robot << "Position: (" << static_cast<int>(robotPosition.x) << ", "
          << static_cast<int>(robotPosition.y) << ")\nHeading: "
          << static_cast<int>(data.amr.getHeading() * 57.2957795f) << " deg\nState: "
          << (data.pathExecution.getState() == PathExecutionState::NotFollowing
                ? "manual" : pathExecutionLabel(data.pathExecution.getState()));
    sections.push_back({"Robot State", robot.str()});

    std::ostringstream mode;
    mode << "Mode: " << (data.localizationDrivenNavigation
        ? "Localization-Driven" : "Simulation Truth")
         << "\nPlanning Start: " << data.planningStartSource;
    if (data.localizationDrivenNavigation) {
        mode << "\nLocalization: "
             << localizationStateLabel(data.localizationStatistics.state) << " / "
             << localizationSupportLabel(data.localizationStatistics.support);
    }
    sections.push_back({"Navigation", mode.str()});

    const PathResult& result = data.pathExecution.getResult();
    std::ostringstream planning;
    planning << "Success: " << (result.success ? "yes" : "no") << "\nNodes: "
             << result.nodesExpanded << "\nLength: " << static_cast<int>(result.pathLength)
             << "\nGrid cells: " << result.path.size() << "\nMessage: " << result.message;
    sections.push_back({"Path Planning", planning.str()});

    std::ostringstream execution;
    const std::size_t waypointCount = data.pathExecution.getExecutionWaypoints().size();
    const std::size_t waypointIndex = data.pathExecution.getCurrentWaypointIndex();
    execution << "State: " << pathExecutionLabel(data.pathExecution.getState())
              << "\nWaypoint: " << (waypointCount == 0 ? 0 : waypointIndex + 1)
              << " / " << waypointCount
              << "\nRemaining: " << (waypointIndex < waypointCount ? waypointCount - waypointIndex : 0)
              << "\nStatus / stop reason: " << data.navigationStatusMessage;
    sections.push_back({"Path Execution", execution.str()});
    sections.push_back({
        "Navigation Controls",
        "Enter Plan Path\nCtrl+R Reset Robot\nCtrl+M Toggle Navigation Mode"
    });
    return sections;
}

std::vector<InspectorSection> buildLocalizationSections(const InspectorData& data) {
    std::vector<InspectorSection> sections;
    const LocalizationEstimate& estimate = data.localizationEstimate;
    const LocalizationStatistics& statistics = data.localizationStatistics;

    std::ostringstream primary;
    primary << "State: " << localizationStateLabel(statistics.state)
            << "\nSupport: " << localizationSupportLabel(statistics.support)
            << "\nInitialization: "
            << localizationInitializationLabel(statistics.initialization)
            << "\nReason: " << statistics.explanation;
    sections.push_back({"Primary State", primary.str()});

    std::ostringstream estimateText;
    estimateText << std::fixed << std::setprecision(1);
    if (estimate.valid) {
        estimateText << "X: " << estimate.pose.position.x << "\nY: "
                     << estimate.pose.position.y << "\nYaw: "
                     << estimate.pose.heading * 57.2957795 << " deg";
    } else {
        estimateText << "X: -\nY: -\nYaw: -";
    }
    sections.push_back({"Estimate", estimateText.str()});

    std::ostringstream confidence;
    confidence << std::fixed << std::setprecision(1);
    if (estimate.valid) {
        const double positionSigma = std::sqrt(std::max(
            0.0, estimate.covariance.xx() + estimate.covariance.yy()
        ));
        confidence << "Position sigma: " << positionSigma
                   << "\nHeading sigma: "
                   << std::sqrt(std::max(0.0, estimate.covariance.yawYaw())) * 57.2957795
                   << " deg\nDominant weight: " << statistics.dominantClusterWeight
                   << "\nSecond weight: " << statistics.secondClusterWeight;
    } else {
        confidence << "Position sigma: -\nHeading sigma: -\nDominant weight: "
                   << statistics.dominantClusterWeight << "\nSecond weight: "
                   << statistics.secondClusterWeight;
    }
    sections.push_back({"Confidence", confidence.str()});

    std::ostringstream filter;
    filter << std::fixed << std::setprecision(1)
           << "Particles: " << statistics.particleCount << "\nESS pre / post: "
           << statistics.preResampleEffectiveSampleSize << " / "
           << statistics.effectiveSampleSize << "\nEntropy: " << statistics.particleEntropy
           << "\nClusters: " << statistics.significantClusterCount << " significant / "
           << statistics.clusterCount << " total";
    sections.push_back({"Particle Filter", filter.str()});

    const SensorUpdateResult& sensor = statistics.sensor;
    std::ostringstream sensorText;
    sensorText << std::fixed << std::setprecision(2)
               << "Total / selected: " << sensor.totalBeams << " / " << sensor.selectedBeams
               << "\nUsed / skipped: " << sensor.usedBeams << " / " << sensor.skippedBeams
               << "\nInvalid / max-range: " << sensor.invalidBeams << " / "
               << sensor.maxRangeBeams << "\nQuality / contrast: "
               << sensor.observationQuality << " / " << sensor.likelihoodContrast
               << "\nBeam-skip fallback: " << (sensor.beamSkipFallback ? "yes" : "no")
               << "\nSensor updates: " << statistics.sensorUpdateCount;
    sections.push_back({"Sensor", sensorText.str()});

    std::ostringstream recovery;
    recovery << std::fixed << std::setprecision(3)
             << "Probability: " << statistics.recoveryProbability
             << "\nSlow average: " << statistics.slowWeightAverage
             << "\nFast average: " << statistics.fastWeightAverage;
    sections.push_back({"Recovery", recovery.str()});

    if (data.localizationView.diagnostics) {
        std::ostringstream diagnostics;
        diagnostics << std::fixed << std::setprecision(1)
                    << "Odometry: (" << data.odometryPose.position.x << ", "
                    << data.odometryPose.position.y << ", "
                    << data.odometryPose.heading * 57.2957795 << " deg)\nTruth: ("
                    << data.groundTruthPose.position.x << ", "
                    << data.groundTruthPose.position.y << ", "
                    << data.groundTruthPose.heading * 57.2957795 << " deg)";
        if (estimate.valid) {
            diagnostics << "\nPosition error: " << std::hypot(
                estimate.pose.position.x - data.groundTruthPose.position.x,
                estimate.pose.position.y - data.groundTruthPose.position.y
            ) << "\nHeading error: " << std::abs(normalizeLocalizationAngle(
                estimate.pose.heading - data.groundTruthPose.heading
            )) * 57.2957795 << " deg";
        } else {
            diagnostics << "\nPosition error: -\nHeading error: -";
        }
        diagnostics << "\nHistory: " << data.localizationHistorySize;
        sections.push_back({"Diagnostics (display only)", diagnostics.str()});
    }

    auto onOff = [](bool enabled) { return enabled ? "ON" : "OFF"; };
    std::ostringstream layers;
    layers << "F1 Particles: " << onOff(data.localizationView.particles)
           << "\nF2 LiDAR rays: " << onOff(data.localizationView.lidarRays)
           << "\nF3 LiDAR hits: " << onOff(data.localizationView.lidarHitPoints)
           << "\nF4 AMCL estimate: " << onOff(data.localizationView.estimate)
           << "\nF6 Covariance: " << onOff(data.localizationView.covariance)
           << "\nF7 Odometry: " << onOff(data.localizationView.odometry)
           << "\nF8 Diagnostics: " << onOff(data.localizationView.diagnostics);
    sections.push_back({"Layers", layers.str()});
    sections.push_back({
        "Localization Controls",
        std::string("Ctrl+L Local Reset\nCtrl+G Global Localization\n")
            + "Ctrl+Shift+K Kidnap at cursor"
            + (data.kidnapTestActive ? "\nKidnap test active" : "")
    });
    return sections;
}

std::vector<InspectorSection> buildSlamSections(const InspectorData& data) {
    std::vector<InspectorSection> sections;
    const SlamUpdateResult& update = data.slamUpdate;
    std::ostringstream primary;
    primary << "State: " << slamStateLabel(update.state)
            << "\nReason: " << scanMatchReasonLabel(update.match.reason)
            << "\nPose valid: " << (update.poseValid ? "yes" : "no")
            << "\nMap integrated: " << (update.mapIntegrated ? "yes" : "no");
    sections.push_back({"SLAM State", primary.str()});

    std::ostringstream pose;
    pose << std::fixed << std::setprecision(1);
    if (update.poseValid) {
        pose << "Frame: SLAM local (truth-aligned for display only)\nCorrected: ("
             << update.pose.position.x << ", "
             << update.pose.position.y << ", "
             << update.pose.heading * 57.2957795 << " deg)\nPredicted: ("
             << update.predictedPose.position.x << ", "
             << update.predictedPose.position.y << ", "
             << update.predictedPose.heading * 57.2957795 << " deg)";
    } else {
        pose << "Frame: SLAM local (truth-aligned for display only)\n"
             << "Corrected: -\nPredicted: -";
    }
    sections.push_back({"Local Pose Estimate", pose.str()});

    std::ostringstream match;
    match << std::fixed << std::setprecision(3)
          << "Accepted: " << (update.match.accepted ? "yes" : "no")
          << "\nScore: " << update.match.score
          << "\nCorrection: (" << update.match.correctionX << ", "
          << update.match.correctionY << ", "
          << update.match.correctionYaw * 57.2957795 << " deg)\nBeams selected / used: "
          << update.match.selectedBeams << " / " << update.match.usedBeams
          << "\nCandidates coarse / fine: " << update.match.coarseCandidates
          << " / " << update.match.fineCandidates;
    sections.push_back({"Scan Matching", match.str()});

    std::ostringstream map;
    map << "Unknown: " << data.slamMapStatistics.unknownCells
        << "\nFree: " << data.slamMapStatistics.freeCells
        << "\nOccupied: " << data.slamMapStatistics.occupiedCells
        << "\nRevision: " << data.slamMapStatistics.revision;
    sections.push_back({"Estimated Occupancy Map", map.str()});

    std::ostringstream lifecycle;
    lifecycle << "Accepted updates: " << update.acceptedUpdates
              << "\nRejected updates: " << update.rejectedUpdates
              << "\nConsecutive failures: " << update.consecutiveFailures;
    sections.push_back({"Lifecycle", lifecycle.str()});

    auto onOff = [](bool enabled) { return enabled ? "ON" : "OFF"; };
    std::ostringstream layers;
    layers << "Occupancy map: " << onOff(data.slamView.occupancyMap)
           << "\nCorrected pose: " << onOff(data.slamView.pose)
           << "\nPredicted pose: " << onOff(data.slamView.predictedPose);
    sections.push_back({"SLAM Layers", layers.str()});
    sections.push_back({
        "SLAM Controls",
        "Ctrl+Shift+L Reset SLAM only\nCtrl+R Robot reset resets both estimators"
    });
    return sections;
}

std::vector<InspectorSection> buildSections(InspectorTab tab, const InspectorData& data) {
    switch (tab) {
    case InspectorTab::Navigation: return buildNavigationSections(data);
    case InspectorTab::Localization: return buildLocalizationSections(data);
    case InspectorTab::Slam: return buildSlamSections(data);
    case InspectorTab::Map:
    default: return buildMapSections(data);
    }
}
}

void InspectorPanel::setBounds(const sf::FloatRect& bounds) {
    m_bounds = bounds;
    for (std::size_t index = 0; index < m_scrollOffsets.size(); ++index) {
        clampScroll(static_cast<InspectorTab>(index));
    }
}

const sf::FloatRect& InspectorPanel::getBounds() const {
    return m_bounds;
}

sf::FloatRect InspectorPanel::getBodyBounds() const {
    const float footerHeight = visibleFooterHeight(m_bounds);
    return sf::FloatRect(
        sf::Vector2f(m_bounds.position.x, m_bounds.position.y + kHeaderHeight),
        sf::Vector2f(
            m_bounds.size.x,
            std::max(0.0f, m_bounds.size.y - kHeaderHeight - footerHeight)
        )
    );
}

sf::FloatRect InspectorPanel::getFooterBounds() const {
    const float footerHeight = visibleFooterHeight(m_bounds);
    return sf::FloatRect(
        sf::Vector2f(
            m_bounds.position.x,
            m_bounds.position.y + m_bounds.size.y - footerHeight
        ),
        sf::Vector2f(m_bounds.size.x, footerHeight)
    );
}

sf::FloatRect InspectorPanel::getTabBounds(InspectorTab tab) const {
    const float tabWidth = m_bounds.size.x / 4.0f;
    return sf::FloatRect(
        sf::Vector2f(
            m_bounds.position.x + tabWidth * static_cast<float>(tabIndex(tab)),
            m_bounds.position.y + 50.0f
        ),
        sf::Vector2f(tabWidth, 36.0f)
    );
}

bool InspectorPanel::contains(const sf::Vector2i& pixelPosition) const {
    return containsPixel(m_bounds, pixelPosition);
}

bool InspectorPanel::handleClick(const sf::Vector2i& pixelPosition) {
    if (!contains(pixelPosition)) {
        return false;
    }
    for (InspectorTab tab : {
            InspectorTab::Map, InspectorTab::Navigation,
            InspectorTab::Localization, InspectorTab::Slam}) {
        if (containsPixel(getTabBounds(tab), pixelPosition)) {
            m_activeTab = tab;
            clampScroll(m_activeTab);
            break;
        }
    }
    return true;
}

InspectorTab InspectorPanel::getActiveTab() const {
    return m_activeTab;
}

float InspectorPanel::getScrollOffset(InspectorTab tab) const {
    return m_scrollOffsets[tabIndex(tab)];
}

void InspectorPanel::setContentHeight(InspectorTab tab, float height) {
    m_contentHeights[tabIndex(tab)] = std::max(0.0f, height);
    clampScroll(tab);
}

void InspectorPanel::scroll(float delta) {
    float& offset = m_scrollOffsets[tabIndex(m_activeTab)];
    offset += delta > 0.0f ? kScrollStep : delta < 0.0f ? -kScrollStep : 0.0f;
    clampScroll(m_activeTab);
}

void InspectorPanel::draw(
    sf::RenderWindow& window,
    const sf::Font& font,
    bool hasFont,
    const InspectorData& data
) {
    const sf::View previousView = window.getView();
    sf::RectangleShape background(m_bounds.size);
    background.setPosition(m_bounds.position);
    background.setFillColor(sf::Color(244, 246, 248));
    background.setOutlineThickness(1.0f);
    background.setOutlineColor(sf::Color(165, 171, 178));
    window.draw(background);
    if (!hasFont || m_bounds.size.x <= 0.0f || m_bounds.size.y <= 0.0f) {
        return;
    }

    sf::Text title(font, "Inspector", 23);
    title.setStyle(sf::Text::Bold);
    title.setFillColor(sf::Color(38, 43, 48));
    title.setPosition(m_bounds.position + sf::Vector2f(kPadding, 12.0f));
    window.draw(title);

    const std::array<std::pair<InspectorTab, const char*>, 4> tabs{{
        {InspectorTab::Map, "Map"},
        {InspectorTab::Navigation, "Navigation"},
        {InspectorTab::Localization, "Localization"},
        {InspectorTab::Slam, "SLAM"}
    }};
    for (const auto& [tab, label] : tabs) {
        const sf::FloatRect tabBounds = getTabBounds(tab);
        const bool active = tab == m_activeTab;
        sf::RectangleShape tabBackground(tabBounds.size);
        tabBackground.setPosition(tabBounds.position);
        tabBackground.setFillColor(active ? sf::Color(70, 130, 180) : sf::Color(226, 230, 234));
        window.draw(tabBackground);
        sf::Text tabText(font, label, 14);
        tabText.setStyle(active ? sf::Text::Bold : sf::Text::Regular);
        tabText.setFillColor(active ? sf::Color::White : sf::Color(67, 72, 78));
        const sf::FloatRect textBounds = tabText.getLocalBounds();
        tabText.setPosition(sf::Vector2f(
            tabBounds.position.x + (tabBounds.size.x - textBounds.size.x) * 0.5f,
            tabBounds.position.y + 8.0f
        ));
        window.draw(tabText);
    }

    const sf::FloatRect bodyBounds = getBodyBounds();
    if (bodyBounds.size.x > 0.0f && bodyBounds.size.y > 0.0f) {
        const sf::Vector2u windowSize = window.getSize();
        const float windowWidth = std::max(1.0f, static_cast<float>(windowSize.x));
        const float windowHeight = std::max(1.0f, static_cast<float>(windowSize.y));
        sf::View bodyView(sf::FloatRect(sf::Vector2f(0.0f, 0.0f), bodyBounds.size));
        bodyView.setViewport(sf::FloatRect(
            sf::Vector2f(bodyBounds.position.x / windowWidth, bodyBounds.position.y / windowHeight),
            sf::Vector2f(bodyBounds.size.x / windowWidth, bodyBounds.size.y / windowHeight)
        ));
        window.setView(bodyView);

        float y = kPadding - getScrollOffset(m_activeTab);
        const float contentWidth = std::max(1.0f, bodyBounds.size.x - 2.0f * kPadding);
        for (const InspectorSection& section : buildSections(m_activeTab, data)) {
            sf::Text heading(font, section.title, 17);
            heading.setStyle(sf::Text::Bold);
            heading.setFillColor(sf::Color(45, 50, 55));
            heading.setPosition(sf::Vector2f(kPadding, y));
            window.draw(heading);
            y += 25.0f;
            for (const std::string& line : wrapText(section.body, font, 14, contentWidth)) {
                sf::Text body(font, line, 14);
                body.setFillColor(section.color);
                body.setPosition(sf::Vector2f(kPadding, y));
                window.draw(body);
                y += 19.0f;
            }
            y += kSectionGap;
        }
        setContentHeight(m_activeTab, y + getScrollOffset(m_activeTab) + kPadding);
        window.setView(previousView);
    }

    const sf::FloatRect footerBounds = getFooterBounds();
    if (footerBounds.size.y <= 0.0f) {
        return;
    }
    const float footerY = footerBounds.position.y;
    sf::RectangleShape footerBackground(footerBounds.size);
    footerBackground.setPosition(footerBounds.position);
    footerBackground.setFillColor(sf::Color(235, 238, 241));
    footerBackground.setOutlineThickness(1.0f);
    footerBackground.setOutlineColor(sf::Color(205, 210, 215));
    window.draw(footerBackground);

    sf::Text legendTitle(font, "Legend", 14);
    legendTitle.setStyle(sf::Text::Bold);
    legendTitle.setFillColor(sf::Color(55, 60, 65));
    legendTitle.setPosition(sf::Vector2f(m_bounds.position.x + kPadding, footerY + 8.0f));
    window.draw(legendTitle);
    const std::array<std::pair<const char*, sf::Color>, 8> legend{{
        {"Robot", sf::Color(50, 110, 190)}, {"Start", sf::Color(20, 150, 60)},
        {"Goal", sf::Color(200, 45, 45)}, {"Path", sf::Color(30, 144, 255)},
        {"Particles", sf::Color(138, 43, 226)}, {"AMCL", sf::Color(255, 20, 147)},
        {"Odometry", sf::Color(255, 140, 0)}, {"LiDAR", sf::Color(0, 145, 165)}
    }};
    const float columnWidth = m_bounds.size.x * 0.5f;
    for (std::size_t index = 0; index < legend.size(); ++index) {
        sf::CircleShape dot(3.0f);
        const float x = m_bounds.position.x + kPadding
            + columnWidth * static_cast<float>(index % 2);
        const float y = footerY + 34.0f + 18.0f * static_cast<float>(index / 2);
        dot.setPosition(sf::Vector2f(x, y + 5.0f));
        dot.setFillColor(legend[index].second);
        window.draw(dot);
        sf::Text label(font, legend[index].first, 12);
        label.setFillColor(sf::Color(70, 74, 78));
        label.setPosition(sf::Vector2f(x + 12.0f, y));
        window.draw(label);
    }
}

std::size_t InspectorPanel::tabIndex(InspectorTab tab) {
    return static_cast<std::size_t>(tab);
}

void InspectorPanel::clampScroll(InspectorTab tab) {
    const float maximum = std::max(
        0.0f,
        m_contentHeights[tabIndex(tab)] - getBodyBounds().size.y
    );
    float& offset = m_scrollOffsets[tabIndex(tab)];
    offset = std::clamp(offset, 0.0f, maximum);
}
