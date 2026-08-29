# Phase: SLAM V1 Architecture

## Ownership

- `SlamOccupancyGrid`: fixed local grid, log odds, observed state, queries,
  coordinate conversion, revision and statistics.
- `OccupancyGridMapper`: deterministic traversal and LiDAR inverse model.
- `CorrelativeScanMatcher`: read-only bounded search and score diagnostics.
- `SlamFrontend`: pose prediction, lifecycle, acceptance, and map integration.
- `SlamVisualization`: read-only cached SFML geometry.
- `Simulator`: sensor synthesis and immutable measurement fan-out.
- `MapData`: simulator truth only.

## Frame rule

SLAM bootstraps its base pose at `(0, 0, 0)`. It never receives the absolute
odometry pose because `OdometrySimulator` seeds that pose from truth. A
display-only initial truth transform may place SLAM-local geometry in the SFML
world. It must never feed inference.

## Sensor orchestration

One accepted simulator pose produces one `OdometryDelta`. Generate one
`LaserScan` when either consumer needs a scan and pass the same value to both
consumers. Use independent deterministic RNG streams for odometry simulation,
LiDAR simulation, and AMCL inference. SLAM V1 itself is deterministic.

## Review gates

- No production Slam API accepts truth or `MapData`.
- `Simulator` coordinates but does not contain mapping or matching algorithms.
- AMCL and SLAM have independent reset/lifecycle state.
- Visualization and Inspector cannot mutate SLAM inference.
- Enabling SLAM cannot alter the delivered odometry or AMCL result sequence.
