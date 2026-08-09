# Project Status

## Current milestone

The first navigation-quality milestone is implemented: runtime planning uses
conservative AMR body clearance, execution removes redundant collinear stops,
and automatic heading changes are progressive.

## Completed behavior

- Start and Goal placement still uses `MapData`'s approved half-open world
  boundary. The observed placement cutoff was a visualization issue: the grid
  extended beyond the editable boundary. Grid rendering is now clipped to the
  world boundary, and outside clicks do not clear the active route.
- `PathPlanner::plan(const MapData&)` retains its approved point/cell behavior.
  Simulator uses a separate clearance-aware overload with the current AMR
  body's conservative enclosing-circle radius.
- For the configured 120 by 60 body, the runtime clearance radius is
  `hypot(60, 30)`, approximately 67.08 world units. Candidate centers are
  rejected when this circle crosses the half-open world boundary or touches an
  obstacle cell AABB. Start and Goal poses and their cell centers use the same
  predicate.
- Inflated occupancy is derived during planning. Persistent obstacles and map
  files remain unchanged.
- `PathResult.path` remains the complete raw four-neighbor A* path.
  `PathExecution` derives a separate execution waypoint sequence by removing
  only intermediate points whose adjacent raw steps are identical unit-cardinal
  directions. Start, Goal, corners, and irregular path segments are preserved.
- Automatic following uses bounded angular change through the differential-
  drive update. Sharp turns rotate in place before translation; aligned motion
  remains frame-time based and clamps the final step. Collision rollback still
  occurs before waypoint advancement. Manual controls are unchanged whenever
  the path is not following.

## Ownership decisions

- `PathPlanner` owns A* and the single clearance-aware traversability predicate;
  it receives a world-unit radius and does not depend on AMR or rendering.
- `AMR` owns its geometry and exposes the conservative body radius plus the
  progressive world-target movement primitive.
- `PathExecution` owns derived transient execution waypoints while retaining
  the authoritative raw `PathResult`.
- Simulator coordinates geometry, planning, execution, rendering, input, and
  collision acceptance. `MapData` remains the authoritative persistent map.

## Verification

- Clean `mingw32-make all` passed on 2026-08-09.
- Clean `mingw32-make test` passed all five suites: 70 PASS, 0 FAIL.
- `CoordinateMapperTests.exe`: 6 PASS, 0 FAIL.
- `MapDataTests.exe`: 16 PASS, 0 FAIL.
- `MapDataFileTests.exe`: 15 PASS, 0 FAIL.
- `PathPlannerTests.exe`: 14 PASS, 0 FAIL.
- `PathExecutionTests.exe`: 19 PASS, 0 FAIL.
- Each test executable was also run directly and passed.
- The desktop executable launched and created an SFML window. The available UI
  automation could not reliably enumerate or attach to that window, so
  placement, clearance, motion, and replanning are not claimed as visually
  verified.

## Known limitations

- The enclosing-circle footprint is deliberately conservative and can reject a
  passage that the rectangular body could traverse at a particular heading.
  Wheels are not included because current runtime collision semantics use the
  body only.
- Only collinear execution points are compressed. There is no line-of-sight
  shortcutting, spline generation, or curvature optimization.
- Automatic motion is deterministic stop-turn-go control, not a continuous-
  curvature trajectory. Goal-pose heading is not executed.
- The current AMR pose still approaches the first Start-cell waypoint without a
  planned approach segment.
- Runtime collision acceptance samples the body's current corners and has no
  swept-volume test, so very large frame times can still tunnel.
- The Makefile does not track header dependencies; header changes require a
  clean rebuild.

## Next smallest step

Perform an interactive desktop acceptance pass covering boundary visualization,
wall/corner clearance, narrow-passage rejection, progressive turning, route
completion, and replanning. Address only a reproduced failure from that pass.

## Important decisions

- The approved one-argument planner API remains unchanged; footprint-aware
  planning is an explicit overload used by Simulator.
- Obstacle inflation is never persisted. The physical map remains authoritative.
- Collinear simplification is execution-only because it preserves the exact raw
  path geometry without introducing a second safety predicate.
