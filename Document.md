# CtrlKine-AMR User Document

This document records what the application can do right now and how to operate it.

## Current Features

- `Select` mode
  Click objects on the simulation canvas to select them.
- `Obstacle` mode
  Place obstacle cells on the grid.
- `Erase` mode
  Delete obstacles, work zones, start pose, or goal pose by clicking them.
- `Set Start` mode
  Place the map start pose and keep its heading.
- `Set Goal` mode
  Place the map goal pose and keep its heading.
- `Work Zone` mode
  Drag to create a grid-aligned work zone rectangle.
- `Pan` mode
  Drag the canvas to move the current view.
- `Zoom`
  Use the mouse wheel over the simulation canvas.
- `Save / Load Map`
  Save the current map to a text file and load it later.
- `Clear Map`
  Remove obstacles, work zones, start pose, and goal pose without deleting the robot.
- `Reset View`
  Restore the default camera pan and zoom.
- `Reset Robot Pose`
  Move the robot back to its default start position and heading.
- `Map Validation`
  Check whether the current map is ready for navigation or simulation.
- `Path Planning Skeleton`
  The project now has a PathPlanner module and planning entry point, but the A* core algorithm is still a teaching TODO scaffold.

## Toolbar

The top toolbar supports both mouse click and keyboard shortcut switching:

- `S` or `1`
  Select
- `O` or `2`
  Obstacle
- `E` or `3`
  Erase
- `T` or `4`
  Set Start
- `G` or `5`
  Set Goal
- `Z` or `6`
  Work Zone
- `P` or `7`
  Pan

## Mouse Controls

- `Left Click`
  Perform the current tool action.
- `Left Drag` in `Pan`
  Move the simulation view.
- `Left Drag` in `Work Zone`
  Create a rectangular work zone.
- `Mouse Wheel`
  Zoom in or out on the simulation canvas.

## Keyboard Controls

- `Up / Down`
  Move the AMR forward or backward.
- `A / D`
  Rotate the AMR left or right.
- `Delete`
  Delete the currently selected obstacle, work zone, start pose, or goal pose.
- `Q`
  Rotate the selected start pose or goal pose counterclockwise in `Select` mode.
- `E`
  Rotate the selected start pose or goal pose clockwise in `Select` mode.
  If no start / goal pose is selected, it still switches to `Erase` mode.
- `Esc`
  Cancel the current tool action.
  If the current mode is `Work Zone`, it also returns to `Select`.
- `F5`
  Save the map to `saved_map.txt`
- `F9`
  Load the map from `saved_map.txt`
- `Ctrl + N`
  Clear the map and clear the current selection.
- `Ctrl + 0`
  Reset the simulation view to the initial pan and zoom.
- `Ctrl + R`
  Reset the robot to its default position and heading.
- `V`
  Run map validation manually and refresh the validation result in Inspector.
- `Enter`
  Trigger the current path planning flow.
  If map validation is `Error`, planning will be blocked.

## Inspector

The right inspector currently shows:

- `Cursor`
  Current world and grid coordinate under the mouse.
- `Map Stats`
  Grid resolution, obstacle count, work zone count, and whether start / goal are set.
- `Map Validation`
  Validation status and validation messages for the current map.
- `Path Planning`
  Current planning result, nodes expanded, path length, and planner message.
- `Selected Object`
  Details for the currently selected object.
- `Robot State`
  Robot position and heading.
- `Status`
  Recent operation result such as save, load, mode switch, or deletion.
- `Action Shortcuts`
  Save / load and reset shortcut reminders.

## Selected Object Details

When an object is selected in `Select` mode:

- `Obstacle`
  Shows grid coordinate and world top-left position.
- `Work Zone`
  Shows position and size.
- `Start Pose`
  Shows position and heading.
  The marker also displays a direction arrow on the canvas.
- `Goal Pose`
  Shows position and heading.
  The marker also displays a direction arrow on the canvas.
- `Robot`
  Shows robot position and heading.

Hit test priority is:

1. `Start Pose`
2. `Goal Pose`
3. `Work Zone`
4. `Obstacle`
5. `Robot`
6. `None`

## Map Validation Rules

The current validator checks:

- `Error`
  - Start pose is not set
  - Goal pose is not set
  - Start pose is on an obstacle
  - Goal pose is on an obstacle
  - Start pose is outside the world boundary
  - Goal pose is outside the world boundary
- `Warning`
  - Robot is colliding with an obstacle
  - Start pose and goal pose are in the same grid cell
  - A work zone is too small or invalid
  - The map has no obstacles
- `Valid`
  - No validation issue is found

## Path Planning Skeleton

The current PathPlanner module is intentionally incomplete.

- `PathPlanner.hpp / PathPlanner.cpp`
  Already exist and are connected to Simulator.
- `Enter`
  Calls the planner when validation is not `Error`.
- `PathResult`
  Currently records:
  - `success`
  - `path`
  - `message`
  - `nodesExpanded`
  - `pathLength`
- `A* implementation`
  Not finished on purpose.
  The main flow and helper functions contain `TODO(student)` comments for the owner to complete:
  - `getStartCell()`
  - `getGoalCell()`
  - `isCellBlocked()`
  - `getNeighbors()`
  - `heuristic()`
  - `reconstructPath()`

At the moment, the planner returns a teaching placeholder result instead of a real A* path.

## Save / Load Format

The map is stored as a human-readable text file named `saved_map.txt`.

Saved data includes:

- `grid_resolution`
- `world_boundary`
- `start_pose`
- `goal_pose`
- `obstacles`
- `work_zones`

`start_pose` and `goal_pose` store both position and heading, so heading is preserved after save and load.

This is not an image export. It stores the actual editable map data.

## Typical Workflow

1. Press `O` and place obstacles.
2. Press `T` and set the start pose.
3. Press `G` and set the goal pose.
4. Press `S` and click the start pose or goal pose if you want to edit its heading.
5. Press `Q / E` to rotate the selected pose.
6. Press `Z` and drag a work zone.
7. Press `Delete` if you want to remove the selected object.
8. Press `F5` to save the map.
9. Press `F9` to load it later.
10. Press `Ctrl + N` when you want to clear the map quickly.
11. Press `Ctrl + 0` when you want to restore the default camera view.
12. Press `Ctrl + R` when you want to move the robot back to its default pose.
13. Press `V` when you want to re-run map validation manually.
14. Press `Enter` when you want to trigger the current path planning skeleton.

## Remaining TODO

- `A* core implementation inside PathPlanner`
- `Path rendering on the simulation canvas`
