# Project Status

## Current milestone

PathPlanner first-version A* skeleton. Goal detection and success-result
handling are implemented; the search still stops before neighbor expansion.

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

## Verification

- `mingw32-make all` passed on 2026-08-09.
- No dedicated PathPlanner unit tests exist yet.

## Known limitations

- Neighbor expansion is not called from the search loop.
- Tentative `gScore` calculation, score relaxation, `cameFrom` updates, and
  neighbor insertion into `openSet` are not implemented.
- No-path failure handling is not implemented.
- The current implementation can therefore complete successfully only when
  the start and goal resolve to the same traversable cell.

## Next smallest step

Implement neighbor expansion for the extracted current cell, including
tentative `gScore` calculation and the approved relaxation/update rules.

## Important decisions

- Planning uses four-neighbor movement with unit grid-step cost and a
  Manhattan heuristic.
- `pathLength` is a grid-step count: path cell count minus one.
- The extracted goal cell is included in `nodesExpanded`.
- Equal-`fScore` tie-breaking and exact message wording are intentionally not
  fixed by the approved specification.
