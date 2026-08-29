# Phase: Simulator and UI Integration

## Simulator

Refactor the current localization update seam minimally into sensor acquisition
plus fan-out. Preserve accepted-motion rollback semantics. Split RNG ownership
into odometry, LiDAR, and AMCL streams with fixed seeds. Generate each scan once
and deliver the same scan to AMCL and SLAM.

SLAM reset is independent of AMCL reset. Map editing never replaces the SLAM
grid. `MapData` remains the source only for sensor simulation, existing
navigation/localization, visualization alignment, and test metrics.

## Visualization

`SlamVisualization` reads a grid snapshot and frontend diagnostics. Cache free
and occupied geometry by grid revision. Draw the estimated map, corrected pose,
and optional predicted pose through a display-only transform established at
SLAM reset. Rendering must not mutate inference.

## Inspector

Add a fourth `SLAM` tab if the existing ownership remains sound. Show state,
reason, local corrected/predicted pose, score/correction, beam and candidate
counts, map statistics, and update/rejection totals. Any truth error must be
clearly labeled display-only and must not be passed back to SLAM.

## Gate

Headless integration tests prove identical sensor fan-out, RNG independence,
independent resets, unchanged `MapData` revision/content, observational UI, and
unchanged truth/AMCL navigation behavior.
