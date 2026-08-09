# Project Status

## Current milestone

PathPlanner first-version A* search. Goal detection, success-result handling,
neighbor expansion, score relaxation, and explicit no-path failure handling
are implemented.

## Completed behavior

- Approved `docs/specs/PathPlannerSpec.md` and synchronized specification index.
- Start and goal poses are converted from `MapData` world positions to
  `GridCoord` through `CoordinateMapper`.
- Boundary and obstacle cells are treated as blocked.
- Four-neighbor candidates are generated.
- Manhattan heuristic is implemented.
- `reconstructPath()` returns the complete start-to-goal path.
- A* state is initialized with `openSet`, empty `cameFrom`, `gScore`, and
  `fScore`; the start node is initialized and inserted into `openSet`.
- `openSet` selection scans `fScore` and removes the lowest-`fScore` current
  cell.
- Extracted cells increment `nodesExpanded`.
- When the extracted current cell is the goal, the planner returns a success
  result with the reconstructed path and grid-step `pathLength`.
- Non-goal current cells expand valid four-neighbor cells with unit movement
  cost.
- Better or previously unknown routes update `cameFrom`, `gScore`, and
  `fScore`, and insert the neighbor into `openSet`.
- When `openSet` is exhausted without reaching the goal, the planner returns
  an explicit no-path failure with an empty path, zero `pathLength`, and the
  completed search's `nodesExpanded` count.
- Dedicated `PathPlanner` tests cover missing poses, blocked endpoints, the
  same-cell path, a reachable path, and explicit no-path failure contracts.

## Verification

- `mingw32-make all` passed on 2026-08-09.
- `build/tests/PathPlannerTests.exe` built and passed: 7 PASS, 0 FAIL.
- `mingw32-make test` ran all four test executables. PathPlanner passed with
  7 PASS, 0 FAIL; the existing suites retain 14 specification failures, so
  the aggregate test target exits non-zero.

## Known limitations

- Existing CoordinateMapper, MapData, and MapDataFile specification failures
  remain; they are outside this task's scope.

## Next smallest step

Investigate the existing CoordinateMapper and MapData specification failures
before extending further PathPlanner coverage.

## Important decisions

- Planning uses four-neighbor movement with unit grid-step cost and a
  Manhattan heuristic.
- `pathLength` is a grid-step count: path cell count minus one.
- The extracted goal cell is included in `nodesExpanded`.
- Equal-`fScore` tie-breaking and exact message wording are intentionally not
  fixed by the approved specification.
