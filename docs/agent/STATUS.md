# Project Status

## Current milestone

The first 2D LiDAR SLAM milestone is implemented through occupancy mapping,
correlative scan matching, frontend lifecycle, Simulator fan-out, visualization,
Inspector integration, deterministic metrics, stress tests, and benchmarks.

This commit is an explicitly requested development checkpoint, not the completed
milestone. Post-review fixes are present in source but have not received the
required final rebuild and clean regression.

## Completed behavior

- `SlamOccupancyGrid` owns a fixed local log-odds grid with separate observed
  state and unknown/free/occupied classification. It is independent of
  `MapData`.
- `OccupancyGridMapper` applies full LiDAR x/y/yaw extrinsics, deterministic
  clipped ray traversal, free-space updates, occupied endpoints, max-range
  free-only behavior, per-scan occupied-over-free resolution, and one revision
  per committed scan.
- `CorrelativeScanMatcher` performs deterministic bounded coarse/fine x/y/yaw
  search against the SLAM map, reports correction/support/candidate diagnostics,
  and does not mutate the map. Reviewer fixes now select physical hits before
  bounded subsampling, require positive occupied-map support, and normalize tie
  corrections by configured search steps.
- `SlamFrontend` consumes only `OdometryDelta` and `LaserScan`, bootstraps at
  local identity, predicts from rot1/signed translation/rot2 odometry, integrates
  only bootstrap or accepted corrections, freezes its map on rejection, and
  implements `Uninitialized`, `Tracking`, and `Lost` with bounded reacquisition.
- `Simulator` has separate deterministic odometry, LiDAR, and AMCL RNG streams.
  A consumer-neutral sensor-frame seam generates one delta and one scan, then
  fans the same immutable frame to independent AMCL and SLAM consumers.
- SLAM reset is independent of AMCL reset. Robot/map lifecycle resets both where
  the simulated robot frame changes.
- `SlamVisualization` caches cell-sized free/occupied geometry plus corrected and
  predicted pose markers. Its world overlay uses a bootstrap-time truth origin
  only for display; the Inspector labels values as SLAM-local and the overlay as
  truth-aligned for display only.
- `InspectorPanel` has a fourth independently scrollable `SLAM` tab showing
  state, reason, local poses, match score/correction, beams, candidates, map
  statistics, lifecycle counters, and layer state.
- `docs/specs/SlamV1Spec.md`, the updated system overview, and the orchestrated
  phase package under `docs/agent/tasks/` define ownership and completion gates.

## Verification performed

- Synchronized baseline at `d5824d7`: clean build and normal regression passed
  177 PASS / 0 FAIL.
- Before post-review fixes: occupancy mapping 10/10, matcher 9/9, frontend 9/9,
  integration 6/6, and normal regression 205 PASS / 0 FAIL.
- Before post-review fixes: SLAM stress passed 4/4. Ideal position RMSE was
  approximately zero, heading RMSE 0 degrees, occupied IoU 0.8056, known-cell
  agreement 1.0, and coverage 0.3583.
- Before post-review fixes: nominal noise completed 5/5 seeds with mean/worst
  position RMSE 8.5264/13.8094, heading RMSE 2.4436/3.4530 degrees, occupied IoU
  0.2893/0.1667, agreement 0.9839, and coverage 0.3618.
- Before post-review fixes: elevated noise completed 5/5 seeds with mean/worst
  position RMSE 9.5709/15.1394, heading RMSE 2.7963/5.6818 degrees, occupied IoU
  0.3310/0.1493, agreement 0.9833, and coverage 0.3618.
- Before post-review fixes: the unoptimized 91-beam 360-degree benchmark reported
  median/p95 milliseconds of 0.7592/0.7727 for occupancy integration,
  2.2344/2.3888 for matching, and 3.0670/3.1847 for a frontend update, with
  729 coarse plus 125 fine candidates.
- Static ground-truth isolation scan found no `AMR`, `MapData`, simulator,
  Inspector, sensor-simulator, or truth dependency in production SLAM inference
  files.
- Architecture, algorithm, and coverage reviewers found no critical issue. The
  source includes fixes for consumer-neutral fan-out, coordinate-rule wording,
  display labeling/bootstrap alignment, cell-sized visualization, large opposing
  rotations, physical-hit sampling, normalized tie breaking, positive map
  support, headless runtime seams, missing-sample accounting, and a rotated
  corridor stress scenario.
- Current-source validation is incomplete. The most recent reviewer rebuild of
  the expanded matcher suite reported 14 PASS / 1 FAIL because a manual test
  fixture placed an exact pi/2 endpoint in column 30 while float beam metadata
  floors the endpoint into column 29. No final build, normal regression, stress,
  or benchmark was run after the latest fixes.

## Known limitations

- The checkpoint contains a known matcher test-fixture failure described above;
  it is not yet a green milestone commit.
- Newly added Simulator fan-out tests, corrected-pose cell-placement test,
  expanded occupancy tests, missing-sample penalties, and rotated corridor stress
  case have not been rebuilt in the current source state.
- The fixed SLAM grid does not expand. There is no loop closure, pose graph,
  graph optimization, ICP, map persistence, or navigation on the SLAM estimate.
- Strict occupied-cell IoU is sensitive to pose/surface-cell shifts under noise;
  known-cell agreement remains much higher and coverage is reported separately.
- Desktop SLAM tab interaction and visual appearance have not received human
  acceptance. Static/headless UI ownership and geometry are covered only by
  tests completed before the latest reviewer fixes.
- The existing localization benchmark uses 91 beams over 270 degrees; only the
  SLAM benchmark is labeled 91-beam 360-degree.

## Next smallest implementation step

Correct the manual matcher fixture so its non-boundary endpoint oracle is
independent of float cardinal rounding. Then rebuild the expanded mapping,
matcher, frontend, Simulator runtime, and integration suites; run the rotated
corridor stress case; run the complete clean normal regression, localization and
SLAM stress targets, and both benchmarks; fix any remaining important finding;
then replace this checkpoint status with final measured evidence.

## Important decisions

- SLAM inference starts at local identity and never receives absolute odometry,
  `MapData`, AMR pose, or ground truth.
- `MapData` remains simulator truth. Test-only metrics and display-only frame
  alignment may read truth but never feed inference.
- AMCL and SLAM consume shared sensor values but own independent lifecycle,
  state, maps, and inference behavior.
- Unknown cells are not treated as free in mapping accuracy, and incomplete pose
  samples may not improve aggregate RMSE.
- This checkpoint is intentionally published with explicit incomplete
  verification because the user requested an immediate commit/push.
