# Project Status

## Current milestone

The first complete path-execution foundation is implemented: successful A*
results are retained, visualized, and followed from start cell through goal
cell.

## Completed behavior

- `PathExecution` owns the current `PathResult`, waypoint index, and compact
  `NotFollowing` / `Following` / `Completed` execution state.
- A successful non-empty plan replaces the active path and begins following;
  a failed, blocked, or empty result cannot begin execution.
- Replanning resets progress, while map replacement/editing and robot reset
  clear the active executable path and its visualization.
- Simulator renders the active path as a cell-center line with waypoint
  markers, using `CoordinateMapper` without modifying `MapData`.
- While following, autonomous waypoint movement owns the AMR update. Movement
  is frame-time based, clamps its final step to prevent overshoot, advances
  only after collision acceptance, and stops in `Completed` after the final
  waypoint.
- Manual AMR controls are unchanged when execution is not following or has
  completed.
- Existing first-version `PathPlanner` behavior is unchanged.

## Ownership decisions

- `PathPlanner` still owns only A* planning and returns `PathResult`.
- Simulator owns `PathExecution` and coordinates planning, movement, and the
  transient path overlay.
- `AMR` owns robot pose/movement and exposes only a world-space `moveToward`
  primitive; it does not know `PathPlanner`, `PathResult`, or grid conversion.
- `Environment` and `MapData` do not own or mutate executable path state.

## Verification

- Clean `mingw32-make all` passed on 2026-08-09.
- `mingw32-make test` passed all five suites: 54 PASS, 0 FAIL.
- `CoordinateMapperTests.exe`: 6 PASS, 0 FAIL.
- `MapDataTests.exe`: 15 PASS, 0 FAIL.
- `MapDataFileTests.exe`: 15 PASS, 0 FAIL.
- `PathPlannerTests.exe`: 7 PASS, 0 FAIL.
- `PathExecutionTests.exe`: 11 PASS, 0 FAIL.
- Each test executable was also run directly and passed.
- GUI runtime capture was unavailable in the current automation session: the
  process launched but exposed no accessible Windows window handle, so path
  visibility and end-to-end in-app motion were not claimed as visually
  verified.

## Known limitations

- The map start pose and current AMR pose are independent. The AMR first moves
  toward the start-cell waypoint; that approach segment is not planned.
- A* remains cell-center/point based without footprint inflation. Runtime AMR
  collision checks can therefore stall on a planner-valid path near an
  obstacle; automatic replanning is not implemented.
- Waypoint following is constant-speed first-version motion. It has no
  acceleration profile, smoothing, or heading-aware trajectory planning.
- The Makefile still does not track header dependencies, so header changes
  require a clean rebuild.

## Next smallest step

Perform an interactive desktop acceptance pass for visible path replacement,
successful completion, and blocked planning. Choose any further navigation
behavior only as a separate approved milestone.

## Important decisions

- The retained planner result is the single authoritative grid path used by
  both execution and visualization; render vertices are only a derived cache.
- Failed planning replaces prior execution state instead of leaving a stale
  route executable.
- Collision rollback occurs before waypoint advancement, so rejected motion
  cannot consume a waypoint.
