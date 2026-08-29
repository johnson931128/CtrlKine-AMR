# Milestone: First Real 2D LiDAR SLAM

## Mission

Implement a deterministic SLAM V1 that estimates pose and an occupancy map from
only `OdometryDelta` and `LaserScan`. Preserve all existing editor, navigation,
AMCL, recovery, persistence, and 360-degree LiDAR behavior.

The main agent owns architecture, production integration, regression, review
fixes, repository status, commit, and push. Read-only subagents may audit
architecture, algorithms, and verification; they do not edit production files.

## Source of truth

Read in this order before implementation:

1. `AGENTS.md`
2. `docs/agent/STATUS.md`
3. `docs/specs/SystemOverview.md`
4. `docs/specs/SlamV1Spec.md`
5. the phase package and relevant code/tests

`MapData` remains simulator ground truth. The SLAM estimate is owned by SLAM.
Production SLAM inference must not include or accept `AMR`, `MapData`,
`Environment`, `Simulator`, `LidarSimulator`, `OdometrySimulator`, or a truth
pose.

## Phase package

- `phases/SLAM_ARCHITECTURE.md`: ownership, frames, sensor fan-out, isolation
- `phases/OCCUPANCY_MAPPING.md`: grid, log odds, ray traversal, inverse model
- `phases/SCAN_MATCHING.md`: deterministic bounded correlative matching
- `phases/SLAM_FRONTEND.md`: lifecycle and integration ordering
- `phases/SLAM_INTEGRATION.md`: Simulator, visualization, Inspector, RNG seams
- `phases/SLAM_VERIFICATION.md`: tests, metrics, stress, benchmarks, reviews

## Required execution order

1. Synchronize and prove the baseline.
2. Complete architecture, algorithm, and verification audits.
3. Approve `SlamV1Spec.md` and this task package.
4. Implement occupancy mapping; run targeted tests and normal regression.
5. Implement scan matching; run targeted tests, benchmark, and regression.
6. Implement the frontend; run targeted tests and regression.
7. Integrate sensor fan-out, visualization, and Inspector; run integration tests
   and regression.
8. Run deterministic trajectory metrics, stress cases, and benchmarks.
9. Perform architecture, algorithm, and test-coverage reviews.
10. Fix important findings and rerun the complete clean verification sequence.
11. Update `docs/agent/STATUS.md`, commit all milestone files together, push,
    and prove local/upstream synchronization.

## Completion gates

The milestone is complete only when all are true:

- the occupancy grid represents unknown/free/occupied with clamped log odds;
- deterministic rays handle hits, max range, invalid beams, clipping, negative
  coordinates, and full sensor extrinsics;
- the matcher searches a bounded x/y/yaw neighborhood, is deterministic, emits
  diagnostics, does not mutate the map, and has a measured benchmark;
- the frontend implements `Uninitialized`, `Tracking`, and `Lost`, bootstrap,
  prediction, correction, guarded integration, invalid/all-max scans, poor
  matches, and large motion;
- Simulator sends identical sensor values to independent AMCL and SLAM
  consumers without changing sensor or AMCL RNG sequences;
- SLAM visualization and the Inspector are observational;
- production SLAM has no ground-truth dependency;
- targeted suites, normal regression, localization stress, SLAM stress, and
  benchmarks complete with zero functional failures;
- position RMSE, heading RMSE, occupied IoU, occupancy agreement, and coverage
  are reported from test-only truth evaluation in the SLAM-local frame;
- architecture, algorithm, and coverage reviews have no unresolved important
  finding.

## Explicit exclusions

Do not add loop closure, pose graphs, graph optimization, ICP, ROS2, DWA, Pure
Pursuit, controller redesign, dynamic obstacles, map persistence for the SLAM
estimate, or navigation on the SLAM map.

## Verification cadence

After each major phase:

```text
targeted build/test
normal regression
```

Final clean sequence:

```text
mingw32-make clean
mingw32-make all
mingw32-make test
mingw32-make test-localization-stress
mingw32-make test-slam-stress
mingw32-make localization-benchmark
mingw32-make slam-benchmark
```

Run each normal executable directly for final per-suite totals. Do not use
hardware-sensitive timing as a functional pass/fail gate; candidate bounds and
finite deterministic results are functional gates.
