#pragma once

#include <SFML/Graphics.hpp>
#include <optional>
#include <random>
#include <string>

#include "AMR.hpp"
#include "AmclLocalizer.hpp"
#include "ApplicationLayout.hpp"
#include "EditorToolbar.hpp"
#include "Environment.hpp"
#include "InspectorPanel.hpp"
#include "LidarSimulator.hpp"
#include "LocalizationConfig.hpp"
#include "LocalizationVisualization.hpp"
#include "MapLikelihoodField.hpp"
#include "MapValidator.hpp"
#include "OdometrySimulator.hpp"
#include "PathExecution.hpp"
#include "SelectedObject.hpp"

struct SimulatorRuntimeTestAccess;

enum class NavigationMode {
    SimulationTruth,
    LocalizationDriven
};

class Simulator {
public:
/* Simulator lifecycle */
Simulator();
void run();

private:
/* Main loop stages */
void processEvents();
void update(float dt);
void render();

/* Configuration loading */
void loadConfig(const std::string& filename);
void loadLocalizationConfig(const std::string& filename);

/* Input and editor event handling */
void handleEditorHotkeys(const sf::Event& event);
bool handleToolbarClick(const sf::Vector2i& pixelPos);

/* UI rendering */
void drawActivePath();

/* UI layout and viewport management */
void updateCursorPreview();
void updateWindowLayout();

/* Map persistence */
void saveMap();
void loadMap();

/* Reset actions */
void clearMap();
void resetView();
void resetRobotPose();
bool synchronizeRobotToStartPose();
void resetLocalizationForCurrentPose(bool initializeFromStart);
void resetLocalizationOnly();
void globalLocalization();
void runKidnapTest();
void updateLocalization();
void rebuildLocalizationVisualization();

/* Validation and path planning */
void updateValidationResult(bool updateStatusMessage = false);
void runPathPlanning();
void clearPathExecution();
void rebuildPathVisualization();

/* Object selection and editing */
void selectObjectAt(const sf::Vector2f& worldPos);
void clearSelection();
bool deleteSelectedObject();
bool rotateSelectedHeading(float deltaRadians);

/* Headless runtime integration seams */
static bool applyConfiguredStartPose(
    const MapData& mapData,
    AMR& amr,
    PathExecution& pathExecution
);
static void applyRobotReset(
    const MapData& mapData,
    const sf::Vector2f& defaultPosition,
    AMR& amr,
    PathExecution& pathExecution
);
static bool isRobotAtInitialWaypoint(
    const MapData& mapData,
    const AMR& amr,
    const PathExecution& pathExecution
);
static bool isObservedPoseAtInitialWaypoint(
    const MapData& mapData,
    const Pose2D& observedPose,
    const PathExecution& pathExecution
);
static bool localizationPassesNavigationGate(
    const LocalizationEstimate& estimate,
    const LocalizationStatistics& statistics,
    const AmclConfig& config,
    std::string& reason
);
static bool applyLocalizationDrivenCommand(
    const Pose2D& observedPose,
    const sf::Vector2f& target,
    float dt,
    float maxSpeed,
    float maxAngularSpeed,
    float trackWidth,
    AMR& amr
);
static OdometryDelta observeAcceptedMotion(
    const Pose2D& acceptedPose,
    OdometrySimulator& odometrySimulator,
    AmclLocalizer& localizer,
    std::mt19937& randomEngine
);
static bool resetLocalizationState(
    const MapData& mapData,
    const Pose2D& acceptedPose,
    bool initializeFromStart,
    OdometrySimulator& odometrySimulator,
    AmclLocalizer& localizer,
    std::mt19937& randomEngine
);

friend struct SimulatorRuntimeTestAccess;

/* Main window and rendering views */
sf::RenderWindow m_window;
sf::View m_uiView;
sf::View m_simView;
sf::View m_defaultSimView;

/* Toolbar and Inspector UI resources */
sf::Font m_uiFont;
bool m_hasUiFont;
ApplicationLayout m_layout;
EditorToolbar m_toolbar;
InspectorPanel m_inspector;

/* Core simulation modules */
AMRConfig m_amrConfig;
AMR m_amr;
Environment m_env;

/* Localization simulation and inference */
AmclConfig m_amclConfig;
LidarConfig m_lidarConfig;
OdometryConfig m_odometryConfig;
std::mt19937 m_localizationRng;
LidarSimulator m_lidarSimulator;
OdometrySimulator m_odometrySimulator;
MapLikelihoodField m_likelihoodField;
AmclLocalizer m_localizer;
LaserScan m_currentScan;
Pose2D m_scanGroundTruthPose;
LocalizationVisualization m_localizationVisualization;
bool m_kidnapTestActive = false;
NavigationMode m_navigationMode = NavigationMode::SimulationTruth;
std::string m_planningStartSource = "Map Start";
std::string m_navigationStatusMessage = "Idle; plan a path to begin.";

/* Timing and camera interaction state */
sf::Clock m_clock;
bool m_isPanning;
sf::Vector2i m_lastPanPixel;

/* Cursor and viewport state */
std::optional<sf::Vector2f> m_hoverWorldPos;
sf::FloatRect m_simViewportRect;

/* Default reset state */
sf::Vector2f m_defaultRobotPosition;

/* Map file and UI status state */
std::string m_mapFilename;
std::string m_statusMessage;

/* Editor selection state */
SelectedObject m_selectedObject;

/* Validation and path-planning results */
ValidationResult m_validationResult;
PathExecution m_pathExecution;
sf::VertexArray m_pathVertices;

};
