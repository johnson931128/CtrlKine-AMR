# Project Status

## Current milestone

Start Pose and runtime AMR pose semantics are synchronized. Autonomous path
execution now begins at the configured Start Pose without an implicit approach
segment.

## Completed behavior

- `MapData` remains the authoritative owner of configured Start and Goal poses.
- Creating, replacing, or rotating Start through the editor immediately applies
  its world position and heading to the AMR. Loading a map with Start also
  applies that pose.
- Planning reapplies configured Start before validation and A* so manual runtime
  movement cannot leave execution starting from a stale AMR pose.
- Every Start synchronization clears `PathExecution`, resets waypoint progress,
  clears path visualization, and refreshes validation. Changing Start therefore
  requires a new planning request.
- `AMR::setPose()` updates runtime position and heading and synchronizes body and
  wheel render shapes. Collision geometry queries use the same pose.
- Ctrl+R resets to configured Start position and heading when Start exists.
  Without Start it retains the previous default `(400, 400)` position and zero
  heading. Reset preserves map configuration and clears execution and route
  visualization.
- Start remains stored at its exact configured world position. On the first
  execution update only, an AMR already in the raw path's Start cell is accepted
  as having reached waypoint zero without calling `moveToward()`. The existing
  collision gate still runs before waypoint advancement. Later waypoints retain
  exact center-target behavior.
- `PathResult.path`, collinear execution-waypoint compression, clearance-aware
  planning, progressive turning, and completion behavior are unchanged.

## Ownership decisions

- Simulator owns the synchronization, reset policy, initial-waypoint acceptance,
  route invalidation, visualization clearing, and validation refresh.
- AMR only owns and applies its runtime pose and geometry synchronization.
- PathPlanner still reads Start from `MapData`; no planner behavior or raw path
  semantics changed.
- PathExecution remains responsible only for transient path state and waypoint
  progress; no AMR or coordinate-mapping dependency was added.

## Verification

- Clean `mingw32-make clean`, `mingw32-make all`, and `mingw32-make test`
  passed on 2026-08-09.
- All six test executables were also run directly and passed.
- `CoordinateMapperTests.exe`: 6 PASS, 0 FAIL.
- `MapDataTests.exe`: 16 PASS, 0 FAIL.
- `MapDataFileTests.exe`: 15 PASS, 0 FAIL.
- `PathPlannerTests.exe`: 14 PASS, 0 FAIL.
- `PathExecutionTests.exe`: 19 PASS, 0 FAIL.
- `SimulatorRuntimeTests.exe`: 9 PASS, 0 FAIL.
- Total automated result: 79 PASS, 0 FAIL. The previous 70-test baseline is
  preserved.
- The desktop executable launched. The available Windows automation failed to
  enumerate or attach to the SFML window (`0x80070003`), so Start placement,
  heading rotation, route clearing, reset, and motion are not claimed as
  visually verified.

## Known limitations

- The enclosing-circle footprint remains deliberately conservative and can
  reject a passage that the rectangular body could traverse at a particular
  heading. Wheels are not included because runtime collision semantics use the
  body only.
- Only collinear execution points are compressed. There is no line-of-sight
  shortcutting, spline generation, or curvature optimization.
- Automatic motion remains deterministic stop-turn-go control. Goal-pose
  heading is not executed.
- Runtime collision acceptance samples the body's current corners and has no
  swept-volume test, so very large frame times can still tunnel.
- The Makefile does not track header dependencies; header changes require a
  clean rebuild.
- The exact interactive reproduction still needs a human-observed desktop pass
  because window automation could not capture this SFML window.

## Next smallest step

Perform the exact desktop acceptance scenario for Start placement across an
obstacle wall, then verify Start replacement, Start heading rotation, Ctrl+R,
route completion, progressive turning, and footprint clearance. Address only a
reproduced failure from that pass.

## Important decisions

- Start synchronization occurs immediately on editor mutation and again before
  planning to recover from intervening manual AMR movement.
- An off-center Start is not snapped to the grid-cell center. Only waypoint zero
  uses same-cell acceptance, preventing a residual approach-to-center segment
  without weakening later corner-waypoint behavior.
- Successful map load follows the same configured-Start semantics as editor
  placement. Map clear or Start deletion does not teleport the AMR; Ctrl+R uses
  the default pose when no Start exists.
