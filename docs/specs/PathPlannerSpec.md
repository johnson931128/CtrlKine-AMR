Status: Approved
Last Updated: 2026-08-08

# Path Planner Specification

## 1. Scope

This specification defines the first verifiable behavior of
`PathPlanner::plan(const MapData&)` for CtrlKine-AMR.

The first version is a grid-based A* planner with:

* Four-neighbor movement
* Unit cost for each grid move
* Manhattan heuristic
* Start and goal cells derived from `MapData` poses
* World-boundary and obstacle blocking
* A complete grid-cell path on success
* An explicit failure result when planning cannot produce a path

The planner reads map state and returns a `PathResult`. It does not own or
modify `MapData`, and it does not depend on rendering or editor input.

This version does not define 8-neighbor movement, diagonal movement, robot
footprint inflation, dynamic obstacles, path smoothing, replanning, or cost
maps.

## 2. Inputs and Coordinate Conversion

### PP-001 MapData is the planning input

`PathPlanner::plan()` shall use the supplied `MapData` as the authoritative
source for:

* Grid resolution and coordinate conversion
* World boundary
* Obstacles
* Robot start pose
* Robot goal pose

The planner shall not read start or goal values from renderer, editor, or AMR
state.

### PP-002 Start cell

The planner shall read the optional start pose from
`MapData::getRobotStartPose()`.

If the start pose is absent, planning shall fail.

If present, only the pose position is used. The pose heading is not part of
the first-version planning state.

The position shall be converted to a `GridCoord` through
`MapData::getMapper().worldToGrid()`. The conversion therefore follows the
floor semantics defined by `MapCoordinateSpec.md`.

### PP-003 Goal cell

The planner shall read the optional goal pose from
`MapData::getRobotGoalPose()`.

If the goal pose is absent, planning shall fail.

If present, only the pose position is used. The pose heading is not part of
the first-version planning state.

The position shall be converted to a `GridCoord` through
`MapData::getMapper().worldToGrid()`.

### PP-004 Same start and goal cell

When the converted start and goal cells are the same and the cell is not
blocked, planning shall succeed with a path containing exactly that one cell.

The path length for this path is zero.

## 3. Grid and Search Rules

### PP-005 Grid state

The planner shall search over `GridCoord` cells. A cell is identified by its
`col` and `row` values; world-space pose heading is not part of a search
state.

### PP-006 Four-neighbor movement

From a cell `(col, row)`, the only candidate neighbors are:

* `(col + 1, row)`
* `(col - 1, row)`
* `(col, row + 1)`
* `(col, row - 1)`

Diagonal cells shall never be generated. Each accepted neighbor transition
has a cost of `1` grid step.

### PP-007 Manhattan heuristic

For a current cell `from` and goal cell `to`, the heuristic shall be:

`h(from, to) = abs(from.col - to.col) + abs(from.row - to.row)`

The A* priority value shall be based on:

`f(cell) = g(cell) + h(cell, goal)`

where `g(cell)` is the accumulated number of four-neighbor moves from the
start cell.

Equal `fScore` values do not require a deterministic tie-breaking rule. The
implementation may choose its own tie-breaking behavior.

### PP-008 Path reconstruction

When the goal is reached, the returned path shall be reconstructed in forward
order from start to goal, inclusive.

For a path with more than one cell:

* The first cell shall equal the converted start cell.
* The last cell shall equal the converted goal cell.
* Every adjacent pair shall be four-neighbor cells.
* No path cell shall be blocked.

When multiple shortest paths exist, any valid shortest path satisfies this
specification. Tests shall not require one unique path sequence unless the
test map has only one shortest path.

## 4. Blocked Cells

### PP-009 Boundary blocking

A cell shall be blocked when the world-space center of that cell is outside
`MapData`'s world boundary.

The center shall be obtained through
`MapData::getMapper().gridToWorldCenter(cell)`, and containment shall follow
the half-open boundary rules in `MapCoordinateSpec.md`:

* `left <= x < right`
* `top <= y < bottom`

Boundary-blocked cells shall not be expanded or included in a successful
path.

### PP-010 Obstacle blocking

A cell shall be blocked when `MapData::isObstacleAt(cell)` is true.

Obstacle blocking is evaluated by logical grid cell, not by a separate world
space collision calculation.

### PP-011 Combined blocking rule

A cell is traversable only when both conditions hold:

1. Its cell center is inside the world boundary.
2. `MapData` contains no obstacle at that `GridCoord`.

The first version shall not treat work zones, robot geometry, dynamic state,
or any cost-map value as a blocking or traversal-cost input.

### PP-012 Start and goal validation

After conversion, the start and goal cells shall be checked with the same
blocking rule as every other cell.

Planning shall fail when either cell is blocked. A blocked start or goal shall
not be returned as a successful path endpoint.

## 5. PathResult Contract

`PathPlanner::plan()` shall return a value whose fields have the following
meanings.

### PP-013 `success`

`success` shall be `true` if and only if a valid path was found.

It shall be `false` when the start pose is missing, the goal pose is missing,
the start or goal cell is blocked, or no traversable path exists.

### PP-014 `path`

On success, `path` shall contain the complete inclusive start-to-goal sequence
of `GridCoord` cells in traversal order.

On failure, `path` shall be empty.

### PP-015 `message`

`message` shall be non-empty for every result and shall be suitable for
displaying the outcome to a user.

For failure, it shall identify the failure category at minimum:

* Missing start pose
* Missing goal pose
* Blocked start cell
* Blocked goal cell
* No traversable path

The exact user-facing wording is intentionally unspecified. Tests shall not
require complete string equality.

### PP-016 `nodesExpanded`

`nodesExpanded` shall report the number of grid cells that A* removes from
the open set and expands by examining their neighbors during this planning
call.

It shall be zero when planning fails before search begins, including missing
poses and blocked start or goal cells. It shall never be negative.

When the goal cell is removed from the open set, it shall be included in
`nodesExpanded` before the search terminates, even if the implementation
immediately returns the reconstructed path.

### PP-017 `pathLength`

`pathLength` shall report the total cost of the returned path using the
first-version unit movement cost.

The required definition is:

* A path with `n` cells has `n - 1` grid-step moves.
* Each move costs `1`.
* A one-cell start-equals-goal path has length `0`.
* A failed result has length `0`.

`pathLength` is always expressed as a grid-step count. It shall not be
reported in world-distance units.

## 6. Failure Behavior

### PP-018 Missing pose

Missing start or goal pose shall produce an explicit failure result with:

* `success == false`
* Empty `path`
* Non-empty `message` identifying the missing pose
* `nodesExpanded == 0`
* `pathLength == 0`

### PP-019 Blocked endpoint

A blocked start or goal cell shall produce an explicit failure result with:

* `success == false`
* Empty `path`
* Non-empty `message` identifying the blocked endpoint
* `nodesExpanded == 0`
* `pathLength == 0`

### PP-020 No traversable path

If both endpoints are present and traversable but A* exhausts the reachable
search space without reaching the goal, the planner shall return an explicit
failure result with:

* `success == false`
* Empty `path`
* Non-empty `message` identifying that no path exists
* A non-negative `nodesExpanded` count for the completed search
* `pathLength == 0`

## 7. Out of Scope

This specification does not define:

* 8-neighbor or diagonal movement
* Robot footprint inflation
* Dynamic obstacles
* Path smoothing
* Replanning
* Cost maps
* Work-zone traversal costs or blocking
* Heading-aware planning
* Runtime rendering or path visualization
