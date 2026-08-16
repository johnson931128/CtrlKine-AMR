# Project Status

## Current milestone

The pre-SLAM architecture, desktop UI, and full-circle LiDAR milestone is
implemented. Existing map editing, navigation, localization, recovery, and the
optional localization-driven navigation mode remain available; SLAM itself was
not added.

## Completed behavior

- `Simulator` remains the application lifecycle and subsystem coordinator, but
  no longer formats the Inspector, owns tab/scroll state, duplicates toolbar
  geometry, or contains localization/LiDAR overlay drawing math.
- `ApplicationLayout` owns deterministic toolbar, simulation viewport, and
  Inspector geometry. The default window is 1440x900 with a 360-pixel Inspector;
  resize calculations remain nonnegative and preserve the simulation view
  center/zoom. Very short windows hide the fixed legend footer instead of
  allowing it to cover the tabs.
- `EditorToolbar` owns the seven editor-button rectangles, labels, active
  styling, drawing, and hit testing. Drawn and clickable geometry now share one
  source.
- `InspectorPanel` owns Map, Navigation, and Localization tab selection,
  independent per-tab scroll offsets, fixed tab/header presentation, grouped
  section formatting, and the compact legend footer. Inspector clicks and wheel
  input are isolated from map editing and viewport zoom.
- Map shows cursor, statistics, validation, selection, Start/Goal, controls, and
  application status. Navigation shows truth robot state, navigation mode,
  planning source/result, waypoint progress, and a dedicated stop/status reason.
  Localization groups primary state, estimate, confidence, particle filter,
  sensor accounting, recovery, display-only truth/odometry diagnostics, layers,
  and controls.
- `LocalizationVisualization` observationally owns the particle cache, LiDAR
  rays/hit points, AMCL marker/heading, covariance ellipse, odometry marker, and
  localization layer options. It does not mutate inference state or consume
  inference RNG. Max-range/no-return rays may be shown by F2 but are excluded
  from F3 hit points.
- Editor selection has one authoritative owner in `Simulator`; `Environment`
  receives it read-only for rendering. Start/Goal selection now adds a gold halo
  without replacing their semantic green/red marker colors.
- Default LiDAR is 91 beams over a full 360 degrees. A full scan uses
  `angle(i) = -pi + i * (2*pi/N)` for `i = 0..N-1`, so it contains N unique
  directions and never duplicates the -pi/+pi seam. Partial scans retain their
  prior inclusive-endpoint behavior, and the one-beam forward-scan convention
  remains unchanged.
- `LaserScan` remains the sensor abstraction carrying ranges, angle minimum and
  increment, range bounds, and x/y/yaw extrinsics. Shared geometry helpers
  identify full-circle scans, compute beam angles, classify physical hits, and
  select cyclically uniform beam indices.
- Raw LiDAR beam count, AMCL selected beams, and rendered beams remain separate.
  AMCL and visualization use cyclic full-circle selection without seam bias;
  AMCL remains bounded by `maxBeams`, rendering remains bounded by
  `renderedRayCount`, and simulation/prediction/drawing preserve identical
  sensor extrinsics.
- Truth isolation remains intact: truth is used only to synthesize odometry and
  LiDAR measurements and for display/test diagnostics. AMCL consumes
  `OdometryDelta`, `LaserScan`, and map-derived data. A future SLAM subsystem
  can consume the same measurement boundary without accessing AMR truth,
  Inspector state, or visualization internals.

## Verification

- Clean build: `mingw32-make clean` followed by `mingw32-make all` passed.
- Clean normal regression: 177 PASS, 0 FAIL across ten executables:
  CoordinateMapper 6, MapData 16, MapDataFile 15, PathPlanner 16,
  PathExecution 19, SimulatorRuntime/UI 20, LocalizationSensor/Visualization 34,
  ParticleFilter 35, LocalizationIntegration 10, LocalizationConfig 6.
- All ten normal test executables were then run directly; every executable
  returned exit code 0 with the same per-suite PASS/FAIL totals.
- New deterministic coverage includes default/tab switching, independent
  Inspector scroll, small-height footer behavior, layout sizes, shared toolbar
  hit geometry, four/eight-beam full-circle geometry, seam uniqueness,
  heading/yaw rotation, x/y/yaw extrinsics, cyclic AMCL/render sampling,
  full-circle beam skipping, and max-range hit-point exclusion.
- Extended deterministic stress: 4 PASS, 0 FAIL. Local localization succeeded
  10/10 seeds (mean final position error 11.0719); global localization remained
  9/10 within the documented bound with zero false-convergence updates; open maps
  produced 0/10 false convergence; kidnapped recovery succeeded 5/5 seeds.
  Local and kidnapped gates now require those exact recorded baselines.
- Final representative benchmark (milliseconds): field rebuild 0.0108,
  91-beam full-circle LiDAR 0.1300. At 300/1000/2000 particles with 31 beams,
  sensor weighting was 3.4252/11.0652/22.6411, clustering
  0.5217/1.8420/3.6205, and KLD resampling 0.1144/0.3859/0.8454. At 5000
  particles and 31 beams: sensor 55.707, clustering 8.3821, KLD 2.2404.
  At 2000 particles and 91 beams: sensor 62.6348.
- Final architecture review found no critical or important issues. Sensor and UI
  reviewers' concrete findings were fixed and their targeted re-reviews found
  no remaining issue.
- Desktop startup acceptance: the built process launched, remained alive after
  two seconds, and exposed the titled window
  `AMR Physics Simulator & Environment Editor` with a nonzero window handle.
  Windows-control screenshot/input automation could not connect to its native
  pipe (OS error 2), so resize, tabs, scrolling, overlays, hotkeys, and navigation
  are not claimed as visually or interactively verified.

## Known limitations

- Human desktop acceptance is still required for appearance, resize feel, tab
  interaction, Inspector scrolling, map zoom isolation, toolbar/legend
  readability, localization overlays, full-circle LiDAR appearance, reset and
  kidnap controls, and both navigation modes.
- Localization support remains an interpretable first-version heuristic, not a
  formal observability proof. One global stress seed remains safely Tracking
  outside the coarse acquisition bound rather than falsely converging.
- The exact continuous likelihood-field query remains linear in obstacle AABBs;
  sensor weighting is still the dominant high-particle/high-beam cost.
- The Inspector uses a purpose-built SFML text panel rather than a general GUI
  framework. Its footer intentionally disappears at impractically short window
  heights so the tabs remain accessible.
- `LaserScan` models an instantaneous planar scan. Timing metadata and
  rolling-scan motion distortion were intentionally not invented.

## Next smallest meaningful milestone

Perform the human desktop acceptance checklist for the new layout, all three
Inspector tabs, scrolling/zoom isolation, toolbar/legend, localization layers,
360-degree LiDAR, resets/kidnap, and both navigation modes. After that evidence
is recorded, define the first SLAM specification against the existing
`OdometryDelta` + `LaserScan` boundary before implementing any mapping or
scan-matching algorithm.

## Important decisions

- This milestone adds no SLAM, scan matching, map building, path smoothing,
  controller redesign, or alternate planner.
- `MapData`, `AMR`, `PathExecution`, and `AmclLocalizer` remain the
  authoritative owners of persistent map, truth pose, route progress, and
  localization lifecycle respectively.
- Full-circle scans use a cyclic denominator of N; partial scans continue using
  their established inclusive endpoints. No speculative scan timing fields were
  added.
- Ground-truth error remains display/test-only and never influences confidence,
  recovery, resampling, planning gates, or control.
