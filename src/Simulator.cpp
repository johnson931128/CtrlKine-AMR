#include "Simulator.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>
#include <vector>

namespace {
constexpr unsigned int kWindowWidth = 1440;
constexpr unsigned int kWindowHeight = 900;
constexpr float kToolbarHeight = 64.0f;
constexpr float kHeadingStepRadians = 0.17453292f;
const sf::Vector2f kDefaultRobotPosition(400.0f, 400.0f);
const sf::Color kPathColor(30, 144, 255, 220);

Pose2D getAmrPose(const AMR& amr) {
    return Pose2D{amr.getPosition(), amr.getHeading()};
}

bool loadUiFont(sf::Font& font) {
    const std::array<const char*, 4> fontPaths = {
        "C:/Windows/Fonts/segoeui.ttf",
        "C:/Windows/Fonts/arial.ttf",
        "C:/Windows/Fonts/calibri.ttf",
        "C:/Windows/Fonts/consola.ttf"
    };

    for (const char* path : fontPaths) {
        if (font.openFromFile(path)) {
            return true;
        }
    }

    return false;
}

std::string toValidationLabel(ValidationStatus status) {
    switch (status) {
    case ValidationStatus::Valid:
        return "Valid";
    case ValidationStatus::Warning:
        return "Warning";
    case ValidationStatus::Error:
    default:
        return "Error";
    }
}

}

Simulator::Simulator()
    : m_window(sf::VideoMode({kWindowWidth, kWindowHeight}), "AMR Physics Simulator & Environment Editor"),
      m_hasUiFont(false),
      m_amrConfig{100.0f, 60.0f, 30.0f, 10.0f, 70.0f, 80.0f, sf::Color(100, 150, 250), sf::Color(50, 50, 50)},
      m_amr(m_amrConfig, kDefaultRobotPosition),
      m_env(50.0f),
      m_amclConfig(),
      m_lidarConfig(),
      m_odometryConfig(),
      m_localizationRng(m_amclConfig.randomSeed),
      m_lidarSimulator(m_lidarConfig),
      m_odometrySimulator(m_odometryConfig),
      m_localizer(m_amclConfig),
      m_isPanning(false),
      m_lastPanPixel({0, 0}),
      m_simViewportRect(sf::Vector2f(0.0f, kToolbarHeight), sf::Vector2f(1080.0f, 836.0f)),
      m_defaultRobotPosition(kDefaultRobotPosition),
      m_mapFilename("saved_map.txt"),
      m_statusMessage("Ready"),
      m_selectedObject(SelectedObject::none()) {
    loadConfig("config.txt");
    loadLocalizationConfig("localization.cfg");
    m_hasUiFont = loadUiFont(m_uiFont);
    m_amr = AMR(m_amrConfig, m_defaultRobotPosition);

    m_uiView = m_window.getDefaultView();
    m_simView = sf::View(
        sf::Vector2f({400.0f, 400.0f}),
        sf::Vector2f({m_simViewportRect.size.x, m_simViewportRect.size.y})
    );
    updateWindowLayout();

    m_pathVertices.setPrimitiveType(sf::PrimitiveType::LineStrip);
    PathResult initialPathResult;
    initialPathResult.message = "Path planning has not run yet.";
    m_pathExecution.install(std::move(initialPathResult));
    m_likelihoodField.rebuild(m_env.getMapData(), m_amclConfig.likelihoodMaxDistance);
    m_odometrySimulator.reset(getAmrPose(m_amr));
    updateValidationResult();
}

void Simulator::loadLocalizationConfig(const std::string& filename) {
    LocalizationConfigSet config{m_amclConfig, m_lidarConfig, m_odometryConfig};
    std::string error;
    if (!loadLocalizationConfigFile(filename, config, error)) {
        std::cerr << "Warning: " << error << ". Using localization defaults.\n";
        return;
    }
    m_amclConfig = config.amcl;
    m_lidarConfig = config.lidar;
    m_odometryConfig = config.odometry;
    m_localizationRng.seed(m_amclConfig.randomSeed);
    m_lidarSimulator = LidarSimulator(m_lidarConfig);
    m_odometrySimulator = OdometrySimulator(m_odometryConfig);
    m_localizer = AmclLocalizer(m_amclConfig);
}

void Simulator::updateWindowLayout() {
    const sf::Vector2u windowSize = m_window.getSize();
    const float windowWidth = std::max(1.0f, static_cast<float>(windowSize.x));
    const float windowHeight = std::max(1.0f, static_cast<float>(windowSize.y));

    const float previousDefaultWidth = m_defaultSimView.getSize().x > 0.0f ? m_defaultSimView.getSize().x : m_simView.getSize().x;
    const float zoomFactor = previousDefaultWidth > 0.0f ? (m_simView.getSize().x / previousDefaultWidth) : 1.0f;
    const sf::Vector2f currentCenter = m_simView.getCenter();

    m_uiView = sf::View(
        sf::Vector2f(windowWidth * 0.5f, windowHeight * 0.5f),
        sf::Vector2f(windowWidth, windowHeight)
    );

    m_layout = calculateApplicationLayout(windowSize);
    m_simViewportRect = m_layout.simulationViewport;

    const sf::Vector2f defaultSimSize(
        std::max(1.0f, m_simViewportRect.size.x),
        std::max(1.0f, m_simViewportRect.size.y)
    );
    m_defaultSimView = sf::View(sf::Vector2f(400.0f, 400.0f), defaultSimSize);
    m_simView.setCenter(currentCenter);
    m_simView.setSize(sf::Vector2f(defaultSimSize.x * zoomFactor, defaultSimSize.y * zoomFactor));
    m_simView.setViewport(sf::FloatRect(
        {m_simViewportRect.position.x / windowWidth, m_simViewportRect.position.y / windowHeight},
        {m_simViewportRect.size.x / windowWidth, m_simViewportRect.size.y / windowHeight}
    ));

    m_toolbar.setBounds(m_layout.toolbar);

    m_inspector.setBounds(m_layout.inspector);
}

void Simulator::loadConfig(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Warning: Cannot open " << filename << ". Using default configurations.\n";
        return;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string key;
        float value;
        if (iss >> key >> value) {
            if (key == "bodyLength") m_amrConfig.bodyLength = value;
            else if (key == "bodyWidth") m_amrConfig.bodyWidth = value;
            else if (key == "wheelLength") m_amrConfig.wheelLength = value;
            else if (key == "wheelWidth") m_amrConfig.wheelWidth = value;
            else if (key == "trackWidth") m_amrConfig.trackWidth = value;
            else if (key == "wheelBase") m_amrConfig.wheelBase = value;
        }
    }
}

void Simulator::saveMap() {
    m_statusMessage = m_env.saveMapToFile(m_mapFilename)
        ? "Saved map to " + m_mapFilename
        : "Failed to save map";
}

void Simulator::loadMap() {
    if (m_env.loadMapFromFile(m_mapFilename)) {
        clearSelection();
        m_likelihoodField.rebuild(m_env.getMapData(), m_amclConfig.likelihoodMaxDistance);
        if (!synchronizeRobotToStartPose()) {
            clearPathExecution();
            resetLocalizationForCurrentPose(false);
            updateValidationResult();
        }
        m_statusMessage = "Loaded map from " + m_mapFilename;
    } else {
        m_statusMessage = "Failed to load map";
    }
}

void Simulator::clearMap() {
    m_env.clearMap();
    clearPathExecution();
    clearSelection();
    m_likelihoodField.rebuild(m_env.getMapData(), m_amclConfig.likelihoodMaxDistance);
    resetLocalizationForCurrentPose(false);
    updateValidationResult();
    m_statusMessage = "Cleared map";
}

void Simulator::resetView() {
    m_simView = m_defaultSimView;
    m_isPanning = false;
    updateCursorPreview();
    m_statusMessage = "Reset view";
}

void Simulator::resetRobotPose() {
    applyRobotReset(
        m_env.getMapData(),
        m_defaultRobotPosition,
        m_amr,
        m_pathExecution
    );
    m_pathVertices.clear();
    resetLocalizationForCurrentPose(m_env.getMapData().getRobotStartPose().has_value());
    updateValidationResult();
    m_statusMessage = "Reset robot pose";
    m_navigationStatusMessage = "Robot reset; plan a new path.";
}

bool Simulator::synchronizeRobotToStartPose() {
    if (!applyConfiguredStartPose(m_env.getMapData(), m_amr, m_pathExecution)) {
        return false;
    }

    m_pathVertices.clear();
    resetLocalizationForCurrentPose(true);
    updateValidationResult();
    return true;
}

void Simulator::resetLocalizationForCurrentPose(bool initializeFromStart) {
    m_localizationRng.seed(m_amclConfig.randomSeed);
    const Pose2D currentPose = getAmrPose(m_amr);
    resetLocalizationState(
        m_env.getMapData(),
        currentPose,
        initializeFromStart,
        m_odometrySimulator,
        m_localizer,
        m_localizationRng
    );
    m_likelihoodField.rebuildIfNeeded(
        m_env.getMapData(),
        m_amclConfig.likelihoodMaxDistance
    );
    m_currentScan = LaserScan{};
    m_scanGroundTruthPose = currentPose;
    m_kidnapTestActive = false;

    rebuildLocalizationVisualization();
}

void Simulator::resetLocalizationOnly() {
    clearPathExecution();
    m_localizationRng.seed(m_amclConfig.randomSeed);
    const std::optional<Pose2D>& start = m_env.getMapData().getRobotStartPose();
    const Pose2D mean = start.has_value() ? *start : getAmrPose(m_amr);
    const bool initialized = m_localizer.initializeLocal(
        mean, m_env.getMapData(), m_localizationRng
    );
    m_currentScan = LaserScan{};
    m_kidnapTestActive = false;
    rebuildLocalizationVisualization();
    if (initialized) {
        m_statusMessage = "Reset localization only";
    } else {
        m_statusMessage = "Localization reset failed";
    }
}

void Simulator::globalLocalization() {
    clearPathExecution();
    m_localizationRng.seed(m_amclConfig.randomSeed);
    const bool initialized = m_localizer.initializeGlobal(
        m_env.getMapData(), m_localizationRng
    );
    m_currentScan = LaserScan{};
    m_kidnapTestActive = false;
    rebuildLocalizationVisualization();
    if (initialized) {
        m_statusMessage = "Global Localization";
    } else {
        m_statusMessage = "Global localization failed";
    }
}

void Simulator::runKidnapTest() {
    if (!m_hoverWorldPos.has_value()
        || !m_env.isInsideWorldBounds(*m_hoverWorldPos)
        || m_env.isObstacleAt(*m_hoverWorldPos)) {
        m_statusMessage = "Kidnap requires a valid free cursor position";
        return;
    }
    AMR candidate = m_amr;
    candidate.setPose(*m_hoverWorldPos, m_amr.getHeading());
    for (const sf::Vector2f& corner : candidate.getCorners()) {
        if (!m_env.isInsideWorldBounds(corner) || m_env.isObstacleAt(corner)) {
            m_statusMessage = "Kidnap requires the complete robot footprint to be free";
            return;
        }
    }
    clearPathExecution();
    m_amr.setPose(*m_hoverWorldPos, m_amr.getHeading());
    m_odometrySimulator.rebaseGroundTruthReference(getAmrPose(m_amr));
    m_localizer.forceSensorUpdate();
    m_currentScan = LaserScan{};
    m_kidnapTestActive = true;
    m_statusMessage = "Kidnap test active";
}

bool Simulator::applyConfiguredStartPose(
    const MapData& mapData,
    AMR& amr,
    PathExecution& pathExecution
) {
    const std::optional<Pose2D>& startPose = mapData.getRobotStartPose();
    if (!startPose.has_value()) {
        return false;
    }

    amr.setPose(startPose->position, startPose->heading);
    pathExecution.clear();
    return true;
}

void Simulator::applyRobotReset(
    const MapData& mapData,
    const sf::Vector2f& defaultPosition,
    AMR& amr,
    PathExecution& pathExecution
) {
    const std::optional<Pose2D>& startPose = mapData.getRobotStartPose();
    if (startPose.has_value()) {
        amr.setPose(startPose->position, startPose->heading);
    } else {
        amr.setPose(defaultPosition, 0.0f);
    }
    pathExecution.clear();
}

bool Simulator::isRobotAtInitialWaypoint(
    const MapData& mapData,
    const AMR& amr,
    const PathExecution& pathExecution
) {
    if (pathExecution.getCurrentWaypointIndex() != 0) {
        return false;
    }

    const std::optional<GridCoord> waypoint = pathExecution.getCurrentWaypoint();
    return waypoint.has_value()
        && mapData.getMapper().worldToGrid(amr.getPosition()) == *waypoint;
}

bool Simulator::isObservedPoseAtInitialWaypoint(
    const MapData& mapData,
    const Pose2D& observedPose,
    const PathExecution& pathExecution
) {
    if (pathExecution.getCurrentWaypointIndex() != 0) {
        return false;
    }
    const std::optional<GridCoord> waypoint = pathExecution.getCurrentWaypoint();
    return waypoint.has_value()
        && mapData.getMapper().worldToGrid(observedPose.position) == *waypoint;
}

bool Simulator::localizationPassesNavigationGate(
    const LocalizationEstimate& estimate,
    const LocalizationStatistics& statistics,
    const AmclConfig& config,
    std::string& reason
) {
    if (!estimate.valid || !estimate.converged
        || statistics.state != LocalizationState::Converged) {
        reason = "Localization state is not Converged.";
        return false;
    }
    if (statistics.support == LocalizationSupport::Insufficient) {
        reason = "Localization support is insufficient.";
        return false;
    }
    if (statistics.dominantClusterWeight < config.navigationDominantWeight) {
        reason = "Dominant localization hypothesis is too weak.";
        return false;
    }
    const double sigmaX = std::sqrt(std::max(0.0, estimate.covariance.xx()));
    const double sigmaY = std::sqrt(std::max(0.0, estimate.covariance.yy()));
    const double sigmaYaw = std::sqrt(std::max(0.0, estimate.covariance.yawYaw()));
    if (sigmaX > config.navigationPositionStdDev
        || sigmaY > config.navigationPositionStdDev
        || sigmaYaw > config.navigationHeadingStdDev) {
        reason = "Localization uncertainty exceeds the navigation gate.";
        return false;
    }
    reason.clear();
    return true;
}

bool Simulator::applyLocalizationDrivenCommand(
    const Pose2D& observedPose,
    const sf::Vector2f& target,
    float dt,
    float maxSpeed,
    float maxAngularSpeed,
    float trackWidth,
    AMR& amr
) {
    const sf::Vector2f delta = target - observedPose.position;
    const float distance = std::hypot(delta.x, delta.y);
    if (distance <= 0.5f) {
        return true;
    }
    if (dt <= 0.0f || maxSpeed < 0.0f || maxAngularSpeed < 0.0f || trackWidth <= 0.0f) {
        return false;
    }
    const float desiredHeading = std::atan2(delta.y, delta.x);
    const float headingError = static_cast<float>(normalizeLocalizationAngle(
        desiredHeading - observedPose.heading
    ));
    const float omega = std::clamp(
        headingError / dt, -maxAngularSpeed, maxAngularSpeed
    );
    const float linearSpeed = std::abs(headingError) > 0.35f
        ? 0.0f
        : std::min(maxSpeed, distance / dt) * std::max(0.0f, std::cos(headingError));
    const float halfDifference = omega * trackWidth * 0.5f;
    amr.update(dt, linearSpeed + halfDifference, linearSpeed - halfDifference);
    return false;
}

OdometryDelta Simulator::observeAcceptedMotion(
    const Pose2D& acceptedPose,
    OdometrySimulator& odometrySimulator,
    AmclLocalizer& localizer,
    std::mt19937& randomEngine
) {
    const OdometryDelta odometry = odometrySimulator.observe(
        acceptedPose,
        randomEngine
    );
    localizer.accumulateOdometry(odometry);
    return odometry;
}

bool Simulator::resetLocalizationState(
    const MapData& mapData,
    const Pose2D& acceptedPose,
    bool initializeFromStart,
    OdometrySimulator& odometrySimulator,
    AmclLocalizer& localizer,
    std::mt19937& randomEngine
) {
    odometrySimulator.reset(acceptedPose);
    if (initializeFromStart && mapData.getRobotStartPose().has_value()) {
        return localizer.initializeLocal(
            *mapData.getRobotStartPose(),
            mapData,
            randomEngine
        );
    }
    localizer.reset();
    return false;
}

void Simulator::updateValidationResult(bool updateStatusMessage) {
    m_validationResult = MapValidator::validate(m_env.getMapData(), m_amr);

    if (updateStatusMessage) {
        m_statusMessage = "Validation: " + toValidationLabel(m_validationResult.status);
    }
}

void Simulator::updateLocalization() {
    const MapData& mapData = m_env.getMapData();
    const bool geometryChanged = !m_likelihoodField.isValid()
        || m_likelihoodField.getSourceRevision() != mapData.getGeometryRevision();
    m_likelihoodField.rebuildIfNeeded(mapData, m_amclConfig.likelihoodMaxDistance);
    if (geometryChanged) {
        m_localizer.forceSensorUpdate();
    }

    const Pose2D groundTruthPose = getAmrPose(m_amr);
    observeAcceptedMotion(
        groundTruthPose,
        m_odometrySimulator,
        m_localizer,
        m_localizationRng
    );
    if (!m_localizer.needsSensorUpdate()) {
        return;
    }

    m_currentScan = m_lidarSimulator.simulate(
        groundTruthPose,
        mapData,
        m_localizationRng
    );
    m_scanGroundTruthPose = groundTruthPose;
    if (m_localizer.updateWithScan(
            m_currentScan,
            m_likelihoodField,
            mapData,
            m_localizationRng)) {
        m_localizer.appendHistory(m_odometrySimulator.getOdometryPose());
        rebuildLocalizationVisualization();
    }
}

void Simulator::rebuildLocalizationVisualization() {
    m_localizationVisualization.rebuildParticles(m_localizer.getParticles());
}

void Simulator::runPathPlanning() {
    PathResult result;

    if (m_navigationMode == NavigationMode::SimulationTruth) {
        synchronizeRobotToStartPose();
        m_planningStartSource = "Map Start";
    } else {
        std::string reason;
        if (!localizationPassesNavigationGate(
                m_localizer.getEstimate(), m_localizer.getStatistics(), m_amclConfig, reason)) {
            result.message = "Planning blocked: localization not reliable. " + reason;
            m_pathExecution.install(std::move(result));
            rebuildPathVisualization();
            m_statusMessage = "Planning blocked: localization not reliable";
            m_navigationStatusMessage = m_pathExecution.getResult().message;
            return;
        }
        m_planningStartSource = "AMCL Estimate";
    }

    if (m_validationResult.status == ValidationStatus::Error) {
        result.message = "Planning blocked: fix map validation errors first.";
        m_pathExecution.install(std::move(result));
        rebuildPathVisualization();
        m_statusMessage = "Planning blocked";
        m_navigationStatusMessage = m_pathExecution.getResult().message;
        return;
    }

    if (m_navigationMode == NavigationMode::LocalizationDriven) {
        const std::optional<Pose2D>& goal = m_env.getMapData().getRobotGoalPose();
        if (!goal.has_value()) {
            result.message = "Planning failed: goal pose is missing.";
        } else {
            result = PathPlanner::plan(
                m_env.getMapData(),
                m_localizer.getEstimate().pose,
                *goal,
                m_amr.getConservativeBodyRadius()
            );
        }
    } else {
        result = PathPlanner::plan(m_env.getMapData(), m_amr.getConservativeBodyRadius());
    }
    m_pathExecution.install(std::move(result));
    rebuildPathVisualization();
    m_statusMessage = m_pathExecution.getResult().message;
    m_navigationStatusMessage = m_pathExecution.getResult().message;
}

void Simulator::clearPathExecution() {
    m_pathExecution.clear();
    m_pathVertices.clear();
    m_navigationStatusMessage = "Path cleared; plan a new path.";
}

void Simulator::rebuildPathVisualization() {
    m_pathVertices.clear();
    if (!m_pathExecution.hasExecutablePath()) {
        return;
    }

    const CoordinateMapper& mapper = m_env.getMapData().getMapper();
    for (const GridCoord& cell : m_pathExecution.getResult().path) {
        m_pathVertices.append(sf::Vertex{mapper.gridToWorldCenter(cell), kPathColor});
    }
}

void Simulator::handleEditorHotkeys(const sf::Event& event) {
    if (const auto* keyPressed = event.getIf<sf::Event::KeyPressed>()) {
        const bool isCtrlPressed =
            sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LControl)
            || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RControl);

        if (isCtrlPressed) {
            switch (keyPressed->code) {
            case sf::Keyboard::Key::N:
                clearMap();
                return;
            case sf::Keyboard::Key::Num0:
                resetView();
                return;
            case sf::Keyboard::Key::R:
                resetRobotPose();
                return;
            case sf::Keyboard::Key::G:
                globalLocalization();
                return;
            case sf::Keyboard::Key::L:
                resetLocalizationOnly();
                return;
            case sf::Keyboard::Key::K:
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift)
                    || sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift)) {
                    runKidnapTest();
                }
                return;
            case sf::Keyboard::Key::M:
                clearPathExecution();
                m_navigationMode = m_navigationMode == NavigationMode::SimulationTruth
                    ? NavigationMode::LocalizationDriven
                    : NavigationMode::SimulationTruth;
                m_statusMessage = m_navigationMode == NavigationMode::LocalizationDriven
                    ? "Navigation mode: Localization-Driven"
                    : "Navigation mode: Simulation Truth";
                m_navigationStatusMessage = "Navigation mode changed; plan a new path.";
                return;
            default:
                break;
            }
        }

        switch (keyPressed->code) {
        case sf::Keyboard::Key::F1:
            m_localizationVisualization.getOptions().particles =
                !m_localizationVisualization.getOptions().particles;
            rebuildLocalizationVisualization();
            break;
        case sf::Keyboard::Key::F2:
            m_localizationVisualization.getOptions().lidarRays =
                !m_localizationVisualization.getOptions().lidarRays;
            break;
        case sf::Keyboard::Key::F3:
            m_localizationVisualization.getOptions().lidarHitPoints =
                !m_localizationVisualization.getOptions().lidarHitPoints;
            break;
        case sf::Keyboard::Key::F4:
            m_localizationVisualization.getOptions().estimate =
                !m_localizationVisualization.getOptions().estimate;
            break;
        case sf::Keyboard::Key::F6:
            m_localizationVisualization.getOptions().covariance =
                !m_localizationVisualization.getOptions().covariance;
            break;
        case sf::Keyboard::Key::F7:
            m_localizationVisualization.getOptions().odometry =
                !m_localizationVisualization.getOptions().odometry;
            break;
        case sf::Keyboard::Key::F8:
            m_localizationVisualization.getOptions().diagnostics =
                !m_localizationVisualization.getOptions().diagnostics;
            break;
        case sf::Keyboard::Key::Num1:
        case sf::Keyboard::Key::S:
            m_env.setEditorMode(EditorMode::Select);
            break;
        case sf::Keyboard::Key::Num2:
        case sf::Keyboard::Key::O:
            m_env.setEditorMode(EditorMode::PlaceObstacle);
            break;
        case sf::Keyboard::Key::Num3:
            m_env.setEditorMode(EditorMode::DeleteObstacle);
            break;
        case sf::Keyboard::Key::Q:
            if (rotateSelectedHeading(-kHeadingStepRadians)) {
                m_statusMessage = "Rotated selected pose";
            }
            break;
        case sf::Keyboard::Key::E:
            if (rotateSelectedHeading(kHeadingStepRadians)) {
                m_statusMessage = "Rotated selected pose";
            } else {
                m_env.setEditorMode(EditorMode::DeleteObstacle);
            }
            break;
        case sf::Keyboard::Key::Num4:
        case sf::Keyboard::Key::T:
            m_env.setEditorMode(EditorMode::SetStartPose);
            break;
        case sf::Keyboard::Key::Num5:
        case sf::Keyboard::Key::G:
            m_env.setEditorMode(EditorMode::SetGoalPose);
            break;
        case sf::Keyboard::Key::Num6:
        case sf::Keyboard::Key::Z:
            m_env.setEditorMode(EditorMode::DrawWorkZone);
            break;
        case sf::Keyboard::Key::Num7:
        case sf::Keyboard::Key::P:
            m_env.setEditorMode(EditorMode::PanView);
            break;
        case sf::Keyboard::Key::V:
            updateValidationResult(true);
            break;
        case sf::Keyboard::Key::Enter:
            runPathPlanning();
            break;
        case sf::Keyboard::Key::Escape:
            m_env.cancelActiveTool();
            m_isPanning = false;
            if (m_env.getEditorMode() == EditorMode::DrawWorkZone) {
                m_env.setEditorMode(EditorMode::Select);
            }
            if (!m_selectedObject.isNone()) {
                clearSelection();
            }
            break;
        case sf::Keyboard::Key::Delete:
            if (deleteSelectedObject()) {
                m_statusMessage = "Deleted selected object";
            }
            break;
        case sf::Keyboard::Key::F5:
            saveMap();
            break;
        case sf::Keyboard::Key::F9:
            loadMap();
            break;
        default:
            break;
        }
    }
}

bool Simulator::handleToolbarClick(const sf::Vector2i& pixelPos) {
    const std::optional<EditorMode> mode = m_toolbar.hitTest(pixelPos);
    if (mode.has_value()) {
        m_env.setEditorMode(*mode);
        m_statusMessage = "Switched mode";
        return true;
    }
    return false;
}

void Simulator::selectObjectAt(const sf::Vector2f& worldPos) {
    SelectedObject selectedObject = m_env.hitTest(worldPos);
    if (selectedObject.isNone() && m_amr.containsPoint(worldPos)) {
        selectedObject = SelectedObject::robot();
    }

    m_selectedObject = selectedObject;
    m_statusMessage = m_selectedObject.isNone() ? "Selection cleared" : "Selected object";
}

void Simulator::clearSelection() {
    m_selectedObject = SelectedObject::none();
}

bool Simulator::deleteSelectedObject() {
    if (m_selectedObject.isNone() || m_selectedObject.type == SelectedObjectType::Robot) {
        return false;
    }

    const bool deleted = m_env.deleteSelectedObject(m_selectedObject);
    if (deleted) {
        clearPathExecution();
        clearSelection();
        updateValidationResult();
    }
    return deleted;
}

bool Simulator::rotateSelectedHeading(float deltaRadians) {
    if (m_env.getEditorMode() != EditorMode::Select) {
        return false;
    }

    if (m_selectedObject.type != SelectedObjectType::StartPose
        && m_selectedObject.type != SelectedObjectType::GoalPose) {
        return false;
    }

    const bool rotated = m_env.rotateSelectedHeading(m_selectedObject, deltaRadians);
    if (rotated && m_selectedObject.type == SelectedObjectType::StartPose) {
        synchronizeRobotToStartPose();
    }
    return rotated;
}

void Simulator::updateCursorPreview() {
    const sf::Vector2i mousePixel = sf::Mouse::getPosition(m_window);
    const bool insideSimX = mousePixel.x >= static_cast<int>(m_simViewportRect.position.x)
        && mousePixel.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x);
    const bool insideSimY = mousePixel.y >= static_cast<int>(m_simViewportRect.position.y)
        && mousePixel.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y);

    if (insideSimX && insideSimY) {
        m_hoverWorldPos = m_window.mapPixelToCoords(mousePixel, m_simView);
        m_env.setCursorWorldPosition(m_hoverWorldPos);
        return;
    }

    m_hoverWorldPos.reset();
    m_env.setCursorWorldPosition(std::nullopt);
}

void Simulator::run() {
    while (m_window.isOpen()) {
        processEvents();
        const float dt = m_clock.restart().asSeconds();
        update(dt);
        render();
    }
}

void Simulator::processEvents() {
    while (const std::optional<sf::Event> event = m_window.pollEvent()) {
        handleEditorHotkeys(*event);

        if (event->is<sf::Event::Closed>()) {
            m_window.close();
        }

        if (const auto* resized = event->getIf<sf::Event::Resized>()) {
            m_window.setView(m_window.getDefaultView());
            updateWindowLayout();
            (void)resized;
        }

        if (const auto* scroll = event->getIf<sf::Event::MouseWheelScrolled>()) {
            const sf::Vector2i mousePos = scroll->position;
            if (m_inspector.contains(mousePos)) {
                m_inspector.scroll(scroll->delta > 0 ? -1.0f : 1.0f);
            } else if (m_simViewportRect.contains(sf::Vector2f(
                    static_cast<float>(mousePos.x), static_cast<float>(mousePos.y)))) {
                if (scroll->delta > 0) m_simView.zoom(0.9f);
                else if (scroll->delta < 0) m_simView.zoom(1.1f);
            }
        }

        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonPressed>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left
                && m_inspector.handleClick(mouseBtn->position)) {
                continue;
            }
            if (mouseBtn->button == sf::Mouse::Button::Left
                && m_toolbar.getBounds().contains(sf::Vector2f(
                    static_cast<float>(mouseBtn->position.x),
                    static_cast<float>(mouseBtn->position.y)))
                && handleToolbarClick(mouseBtn->position)) {
                continue;
            }

            const bool insideSimArea = mouseBtn->position.x >= static_cast<int>(m_simViewportRect.position.x)
                && mouseBtn->position.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x)
                && mouseBtn->position.y >= static_cast<int>(m_simViewportRect.position.y)
                && mouseBtn->position.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y);
            if (mouseBtn->button == sf::Mouse::Button::Left && insideSimArea) {
                const sf::Vector2f worldPos = m_window.mapPixelToCoords(mouseBtn->position, m_simView);

                if (m_env.getEditorMode() == EditorMode::PanView) {
                    m_isPanning = true;
                    m_lastPanPixel = mouseBtn->position;
                } else if (m_env.getEditorMode() == EditorMode::Select) {
                    selectObjectAt(worldPos);
                } else if (m_env.getEditorMode() == EditorMode::DeleteObstacle) {
                    SelectedObject deletedObject = SelectedObject::none();
                    if (m_env.deleteObjectAt(worldPos, &deletedObject)) {
                        clearPathExecution();
                        if (m_selectedObject.type == deletedObject.type
                            && m_selectedObject.obstacleCoord == deletedObject.obstacleCoord
                            && m_selectedObject.workZoneIndex == deletedObject.workZoneIndex) {
                            clearSelection();
                        }
                        updateValidationResult();
                        m_statusMessage = "Deleted object";
                    }
                } else {
                    if (!m_env.isInsideWorldBounds(worldPos)) {
                        continue;
                    }
                    const EditorMode editorMode = m_env.getEditorMode();
                    m_env.handleLeftMousePressed(worldPos);
                    if (editorMode == EditorMode::SetStartPose) {
                        synchronizeRobotToStartPose();
                    } else {
                        clearPathExecution();
                        updateValidationResult();
                    }
                }
            }
        }

        if (const auto* mouseBtn = event->getIf<sf::Event::MouseButtonReleased>()) {
            if (mouseBtn->button == sf::Mouse::Button::Left) {
                const bool insideSimArea = mouseBtn->position.x >= static_cast<int>(m_simViewportRect.position.x)
                    && mouseBtn->position.x < static_cast<int>(m_simViewportRect.position.x + m_simViewportRect.size.x)
                    && mouseBtn->position.y >= static_cast<int>(m_simViewportRect.position.y)
                    && mouseBtn->position.y < static_cast<int>(m_simViewportRect.position.y + m_simViewportRect.size.y);
                if (insideSimArea) {
                    const sf::Vector2f worldPos = m_window.mapPixelToCoords(mouseBtn->position, m_simView);
                    m_env.handleLeftMouseReleased(worldPos);
                    updateValidationResult();
                }
                m_isPanning = false;
            }
        }

        if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>()) {
            if (m_isPanning && m_env.getEditorMode() == EditorMode::PanView) {
                const sf::Vector2f previousWorld = m_window.mapPixelToCoords(m_lastPanPixel, m_simView);
                const sf::Vector2f currentWorld = m_window.mapPixelToCoords(mouseMoved->position, m_simView);
                m_simView.move(previousWorld - currentWorld);
                m_lastPanPixel = mouseMoved->position;
            }
        }
    }

    updateCursorPreview();
}

void Simulator::update(float dt) {
    const float maxSpeed = 150.0f;
    const float turnRate = 100.0f;
    const float maxAutomaticAngularSpeed = m_amrConfig.trackWidth > 0.0f
        ? (2.0f * turnRate) / m_amrConfig.trackWidth
        : 0.0f;

    AMR backupAmr = m_amr;
    const sf::Vector2f previousPosition = m_amr.getPosition();
    const float previousHeading = m_amr.getHeading();
    bool wasFollowing = m_pathExecution.getState() == PathExecutionState::Following;
    bool reachedWaypoint = false;

    if (wasFollowing && m_navigationMode == NavigationMode::LocalizationDriven) {
        std::string reason;
        if (!localizationPassesNavigationGate(
                m_localizer.getEstimate(), m_localizer.getStatistics(), m_amclConfig, reason)) {
            clearPathExecution();
            m_statusMessage = "Execution stopped: localization lost. " + reason;
            m_navigationStatusMessage = m_statusMessage;
            wasFollowing = false;
        }
    }

    if (wasFollowing) {
        const std::optional<GridCoord> waypoint = m_pathExecution.getCurrentWaypoint();
        if (waypoint.has_value()) {
            const Pose2D observedPose = m_navigationMode == NavigationMode::LocalizationDriven
                ? m_localizer.getEstimate().pose
                : getAmrPose(m_amr);
            const bool atInitialWaypoint = m_navigationMode == NavigationMode::LocalizationDriven
                ? isObservedPoseAtInitialWaypoint(
                    m_env.getMapData(), observedPose, m_pathExecution
                )
                : isRobotAtInitialWaypoint(m_env.getMapData(), m_amr, m_pathExecution);
            if (atInitialWaypoint) {
                reachedWaypoint = true;
            } else {
                const sf::Vector2f target = m_env.getMapData().getMapper().gridToWorldCenter(*waypoint);
                if (m_navigationMode == NavigationMode::LocalizationDriven) {
                    reachedWaypoint = applyLocalizationDrivenCommand(
                        observedPose,
                        target,
                        dt,
                        maxSpeed,
                        maxAutomaticAngularSpeed,
                        m_amrConfig.trackWidth,
                        m_amr
                    );
                } else {
                    reachedWaypoint = m_amr.moveToward(
                        target,
                        dt,
                        maxSpeed,
                        maxAutomaticAngularSpeed,
                        0.5f
                    );
                }
            }
        }
    } else {
        float linearSpeed = 0.0f;
        float turnSpeed = 0.0f;

        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Up)) linearSpeed = maxSpeed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::Down)) linearSpeed = -maxSpeed;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::A)) turnSpeed = -turnRate;
        if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::D)) turnSpeed = turnRate;

        const float vL = linearSpeed + turnSpeed;
        const float vR = linearSpeed - turnSpeed;
        m_amr.update(dt, vL, vR);
    }

    const std::vector<sf::Vector2f> corners = m_amr.getCorners();

    bool collision = false;
    for (const auto& corner : corners) {
        if (!m_env.isInsideWorldBounds(corner) || m_env.isObstacleAt(corner)) {
            collision = true;
            break;
        }
    }

    if (collision) {
        m_amr = backupAmr;
    } else if (wasFollowing && reachedWaypoint) {
        m_pathExecution.advanceWaypoint();
        if (m_pathExecution.getState() == PathExecutionState::Completed) {
            m_statusMessage = "Path following completed.";
            m_navigationStatusMessage = m_statusMessage;
        }
    }

    updateLocalization();

    const sf::Vector2f currentPosition = m_amr.getPosition();
    if (currentPosition.x != previousPosition.x
        || currentPosition.y != previousPosition.y
        || m_amr.getHeading() != previousHeading) {
        updateValidationResult();
    }
}

void Simulator::render() {
    m_window.clear(sf::Color(245, 245, 245));

    m_window.setView(m_simView);
    m_env.draw(m_window, m_simView, m_selectedObject);
    m_localizationVisualization.drawScan(m_window, m_currentScan, m_scanGroundTruthPose);
    m_localizationVisualization.drawBelief(
        m_window,
        m_localizer.getEstimate(),
        m_localizer.getStatistics(),
        m_odometrySimulator.getOdometryPose(),
        m_odometrySimulator.isInitialized()
    );
    drawActivePath();
    m_amr.draw(m_window);

    if (m_selectedObject.type == SelectedObjectType::Robot) {
        const std::vector<sf::Vector2f> robotCorners = m_amr.getCorners();
        sf::VertexArray outline(sf::PrimitiveType::LineStrip, 5);
        for (std::size_t i = 0; i < 4; ++i) {
            outline[i] = sf::Vertex{robotCorners[i], sf::Color(255, 165, 0)};
        }
        outline[4] = sf::Vertex{robotCorners[0], sf::Color(255, 165, 0)};
        m_window.draw(outline);
    }

    m_window.setView(m_uiView);
    m_toolbar.draw(m_window, m_uiFont, m_hasUiFont, m_env.getEditorMode());
    const InspectorData inspectorData{
        m_env.getMapData(),
        m_amr,
        m_validationResult,
        m_pathExecution,
        m_selectedObject,
        m_hoverWorldPos,
        m_localizer.getEstimate(),
        m_localizer.getStatistics(),
        m_odometrySimulator.getOdometryPose(),
        m_odometrySimulator.isInitialized(),
        getAmrPose(m_amr),
        m_localizationVisualization.getOptions(),
        m_localizer.getHistory().size(),
        m_navigationMode == NavigationMode::LocalizationDriven,
        m_planningStartSource,
        m_statusMessage,
        m_navigationStatusMessage,
        m_kidnapTestActive
    };
    m_inspector.draw(m_window, m_uiFont, m_hasUiFont, inspectorData);

    m_window.display();
}

void Simulator::drawActivePath() {
    if (!m_pathExecution.hasExecutablePath()) {
        return;
    }

    if (m_pathVertices.getVertexCount() > 1) {
        m_window.draw(m_pathVertices);
    }

    sf::CircleShape waypointMarker(4.0f);
    waypointMarker.setOrigin(sf::Vector2f(4.0f, 4.0f));
    waypointMarker.setFillColor(kPathColor);

    const CoordinateMapper& mapper = m_env.getMapData().getMapper();
    for (const GridCoord& cell : m_pathExecution.getExecutionWaypoints()) {
        waypointMarker.setPosition(mapper.gridToWorldCenter(cell));
        m_window.draw(waypointMarker);
    }
}
