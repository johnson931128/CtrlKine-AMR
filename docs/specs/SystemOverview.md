# CtrlKine-AMR System Overview

## 1. Purpose

CtrlKine-AMR is a C++ and SFML-based 2D environment editor and AMR simulation platform.

The system provides:

* Grid-based map editing
* Obstacle and work-zone placement
* Start and goal pose configuration
* AMR manual movement
* Collision checking
* Map validation
* Map persistence
* Grid-based path-planning integration
* Sensor-only local 2D LiDAR SLAM

## 2. Main Modules

### Simulator

Owns the application lifecycle and coordinates input, update, rendering, UI, validation, and path-planning requests.

### Environment

Interprets editor operations and renders map-related objects.

### MapData

Stores persistent map state, including obstacles, work zones, world boundary, grid resolution, start pose, and goal pose.

### CoordinateMapper

Converts between world coordinates and grid coordinates.

### AMR

Stores and updates robot position, heading, geometry, and manual movement state.

### MapValidator

Checks whether the current map contains invalid or suspicious states.

### PathPlanner

Reads MapData and produces a PathResult. The first implementation uses grid-based A*.

### SlamFrontend

Consumes only OdometryDelta and LaserScan, estimates a pose in a SLAM-local
frame, and owns the lifecycle that orders prediction, scan matching, and map
integration.

### SlamOccupancyGrid

Stores the SLAM-owned log-odds occupancy estimate. It is independent of MapData,
which remains simulator ground truth.

### CorrelativeScanMatcher

Searches a bounded x/y/yaw neighborhood around an odometry prediction and scores
LaserScan alignment against the SLAM-owned occupancy estimate.

## 3. Main Runtime Flow

Input
→ Simulator event routing
→ Environment or AMR state change
→ Map validation
→ Rendering

Path-planning flow:

Enter key
→ Simulator checks validation result
→ PathPlanner reads MapData
→ PathResult is returned
→ Inspector displays the result

SLAM flow:

OdometryDelta and LaserScan
→ SlamFrontend motion prediction
→ CorrelativeScanMatcher correction
→ SlamOccupancyGrid integration
→ observational visualization and Inspector diagnostics

## 4. Architectural Rules

* Simulator coordinates modules but must not contain the A* algorithm.
* MapData is the authoritative source of editable map state.
* Persistent/editor map coordinate conversion must be performed through
  CoordinateMapper. The SLAM-owned local grid performs its own origin-relative
  conversion because it is independent of MapData.
* Inspector displays results but must not implement validation or planning rules.
* PathPlanner must not depend on rendering or editor input.
* SLAM inference must not access AMR ground truth, MapData, sensor simulators,
  rendering, or Inspector state.
* MapData is simulator ground truth; SlamOccupancyGrid is the independent SLAM
  estimate.
* AMCL and SLAM are independent consumers of shared sensor abstractions.
