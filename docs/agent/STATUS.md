# Project Status

## Current milestone

PathPlanner first-version A* search. Goal detection, success-result handling,
neighbor expansion, and score relaxation are implemented; no-path failure
handling remains.

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

## Verification

- `mingw32-make all` passed on 2026-08-09 after neighbor expansion and score
  relaxation changes.
- No dedicated PathPlanner unit tests exist yet.

## Known limitations

- No-path failure handling is not implemented.
- When `openSet` is exhausted without reaching the goal, the planner still
  returns the existing skeleton failure result rather than a dedicated
  no-path category.

## Next smallest step

Implement explicit no-path failure handling after `openSet` is exhausted.

## Important decisions

- Planning uses four-neighbor movement with unit grid-step cost and a
  Manhattan heuristic.
- `pathLength` is a grid-step count: path cell count minus one.
- The extracted goal cell is included in `nodesExpanded`.
- Equal-`fScore` tie-breaking and exact message wording are intentionally not
  fixed by the approved specification.
