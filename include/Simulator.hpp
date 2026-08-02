#pragma once

#include <SFML/Graphics.hpp>
#include <optional>
#include <string>

#include "AMR.hpp"
#include "Environment.hpp"
#include "MapValidator.hpp"
#include "PathPlanner.hpp"
#include "SelectedObject.hpp"

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

/* Validation and path planning */
void updateValidationResult(bool updateStatusMessage = false);
void runPathPlanning();

/* Object selection and editing */
void selectObjectAt(const sf::Vector2f& worldPos);
void clearSelection();
bool deleteSelectedObject();
bool rotateSelectedHeading(float deltaRadians);
void syncSelectionToEnvironment();

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
PathResult m_pathResult;

};
