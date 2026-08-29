# Phase: Occupancy Mapping

## Required behavior

Implement a fixed-extent SLAM-local grid with configurable origin, resolution,
width, and height. Store an observed bit separately from clamped log odds so an
observed neutral cell is not confused with unknown.

Use floor semantics for world-to-cell conversion, including negative values.
Classify cells with configured free and occupied log-odds thresholds.

For every valid beam, apply full x/y/yaw sensor extrinsics. A physical hit marks
all traversed cells before the hit free and the endpoint occupied. A finite
max-range return marks traversed cells free without an occupied endpoint.
Invalid, non-finite, below-minimum, or malformed scans do not update the grid.

Traversal must be deterministic, clipped to the grid, and explicitly tested at
axis-aligned, diagonal, corner, reverse, negative-coordinate, zero-length, and
boundary-surface cases. Occupied evidence wins if one scan observes a cell as
both free and occupied. Commit one grid revision per integrated scan.

## Gate

Run `SlamOccupancyGridTests` and the complete normal regression before scan
matching begins.
