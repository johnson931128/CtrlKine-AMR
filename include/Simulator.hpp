#pragma once

#include <SFML/Graphics.hpp>
#include <optional>
#include <random>
#include <string>

#include "AMR.hpp"
#include "AmclLocalizer.hpp"
#include "Environment.hpp"
#include "LidarSimulator.hpp"
#include "MapLikelihoodField.hpp"
#include "MapValidator.hpp"
#include "OdometrySimulator.hpp"
#include "PathExecution.hpp"
#include "SelectedObject.hpp"

struct SimulatorRuntimeTestAccess;

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

/* Input and editor event handling */
void handleEditorHotkeys(const sf::Event& event);
bool handleToolbarClick(const sf::Vector2i& pixelPos);

/* UI rendering */
void drawToolbar();
void drawInspector();
void drawActivePath();
void drawLocalization();
void drawLidarScan();

/* UI layout and viewport management */
void updateCursorPreview();
void updateWindowLayout();
void scrollInspector(float delta);

/* Map persistence */
void saveMap();
void loadMap();

/* Reset actions */
void clearMap();
void resetView();
void resetRobotPose();
bool synchronizeRobotToStartPose();
void resetLocalizationForCurrentPose(bool initializeFromStart);
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
void syncSelectionToEnvironment();

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
sf::RectangleShape m_toolbarBg;
sf::RectangleShape m_inspectorBg;
sf::RectangleShape m_divider;
sf::Font m_uiFont;
bool m_hasUiFont;

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
sf::VertexArray m_particleVertices;

/* Timing and camera interaction state */
sf::Clock m_clock;
bool m_isPanning;
sf::Vector2i m_lastPanPixel;

/* Cursor and viewport state */
std::optional<sf::Vector2f> m_hoverWorldPos;
sf::FloatRect m_simViewportRect;

/* Default reset state */
sf::Vector2f m_defaultRobotPosition;

/* Inspector scrolling state */
float m_inspectorScrollOffset;
float m_inspectorContentHeight;

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
