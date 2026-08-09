# Project Status

## Current milestone

The first native AMCL localization vertical slice is complete. Ground truth,
simulated odometry, simulated LiDAR, particle belief, estimated pose, and
estimated covariance are separate runtime concepts.

## Completed behavior

- `AMR` remains the sole ground-truth robot pose. `OdometrySimulator` observes
  only accepted post-collision motion and maintains a distinct noisy odometry
  pose. Rejected motion produces a zero increment.
- `LidarSimulator` produces seeded 2D scans from ground truth using exact
  ray/AABB obstacle intersections, world-boundary termination, configured FOV,
  beam count, range limits, sensor heading, and Gaussian range noise.
- `MapLikelihoodField` derives a bounded grid-vertex distance representation
  from `MapData`, interpolates point queries, treats the world boundary as a
  surface, and rebuilds from geometry revision plus geometry signature. It is
  never persisted into map files.
- One explicit `std::mt19937` owned by `Simulator` supplies LiDAR, odometry,
  particle-motion, initialization, resampling, and recovery randomness. The
  configured default seed is repeatable; tests never use wall-clock seeding.
- `ParticleFilter` supports bounded local-Gaussian and global-free-space
  initialization, differential `rot1/trans/rot2` motion with configurable
  noise and lateral slip, stable likelihood-field weighting, normalization,
  ESS, systematic resampling, adaptive KLD sampling, and random recovery
  injection.
- KLD sampling counts bins occupied by newly drawn particles, uses explicit
  x/y/yaw bin sizes and the `pfErr`/positive-z-score `pfZ` formula, and enforces
  configured minimum and maximum particle counts.
- Recovery tracks prior-weighted slow/fast observation-quality averages.
  Severe mismatch increases free-space random injection; zero recovery alphas
  disable it. Invalid or beamless scans do not consume pending odometry.
- `AmclLocalizer` accumulates odometry until translation/rotation thresholds,
  composes one scan-to-scan motion increment, permits an immediate first scan,
  applies the resample interval/ESS gates, and exposes Uninitialized, Tracking,
  Converged, and Recovering diagnostics without using truth error.
- Estimated x/y use the weighted mean; heading uses a circular mean; the full
  3x3 x/y/yaw covariance uses wrapped yaw residuals and finite-value checks.
- Start placement/replacement/rotation, Ctrl+R, planning synchronization, and
  successful load reset odometry and locally initialize uncertain particles.
  Clear without Start leaves localization uninitialized. Obstacle, resolution,
  or boundary changes rebuild derived likelihood geometry and force a scan.
- Path planning and execution still use `MapData` Start/Goal semantics. The
  localization estimate is observational and is not a navigation input.
- Simulator rendering uses vertex arrays for a low-alpha particle cloud and
  lightweight LiDAR rays, plus a distinct estimated-pose marker. Inspector now
  reports state, count, ESS, estimate, covariance sigmas, ground truth, errors,
  and odometry pose.

## Verification

- Clean `mingw32-make clean`, `mingw32-make all`, and `mingw32-make test`
  passed on 2026-08-09.
- All nine test executables were then run directly and passed.
- Existing baseline remained 79 PASS, 0 FAIL:
  - `CoordinateMapperTests.exe`: 6 PASS.
  - `MapDataTests.exe`: 16 PASS.
  - `MapDataFileTests.exe`: 15 PASS.
  - `PathPlannerTests.exe`: 14 PASS.
  - `PathExecutionTests.exe`: 19 PASS.
  - Existing `SimulatorRuntimeTests.exe` cases: 9 PASS.
- New localization coverage is 41 PASS, 0 FAIL:
  - Localization runtime seams added to `SimulatorRuntimeTests.exe`: 3 PASS.
  - `LocalizationSensorTests.exe`: 18 PASS.
  - `ParticleFilterTests.exe`: 17 PASS.
  - `LocalizationIntegrationTests.exe`: 3 PASS.
- Total automated result: 120 PASS, 0 FAIL.
- Deterministic convergence passed with meaningful initial uncertainty,
  noisy odometry, repeated scans, decreasing covariance, bounded particles,
  and final Converged state.
- Deterministic moving tracking passed: raw odometry diverged measurably while
  LiDAR-corrected AMCL finished closer to truth and never copied truth directly.
- Deterministic kidnapped-robot recovery passed: the unreset filter entered a
  genuine recovery phase, injected global hypotheses, found the distant pose,
  and reconverged within 60 bounded updates.
- The complete three-scenario localization integration executable, using
  300-2200 particles and 41-51 selected beams, completed in about 304 ms in the
  measured headless run. No quadratic resampling or per-particle map rebuild is
  present.
- The desktop executable launched, remained responsive, and exposed the
  expected window title. Windows automation could not enumerate/capture the
  SFML window (`0x80070003`), so particle, LiDAR, estimate, Inspector, editing,
  reset, and navigation visuals are not claimed as observed.

## Ownership decisions

- `Simulator` owns the single RNG and coordinates truth-derived sensor
  simulation, accepted-motion odometry, map-derived rebuilds, AMCL updates,
  rendering caches, and truth-error diagnostics.
- `OdometrySimulator` and `LidarSimulator` may read ground truth only to create
  simulated measurements. `AmclLocalizer` and `ParticleFilter` receive no
  `AMR` reference and cannot copy its pose.
- `MapData` remains authoritative persistent geometry; geometry revision is
  non-persisted invalidation metadata.
- `MapLikelihoodField` is derived read-only localization state. Probabilistic
  logic has no rendering dependency; SFML drawing remains in `Simulator`.

## Known limitations

- The likelihood field samples exact obstacle-surface distance at grid vertices
  and bilinearly interpolates between them; it is a first-version approximation
  at the map grid resolution rather than a sub-cell Euclidean distance transform.
- Default localization parameters and the deterministic production seed are
  compiled defaults; there is no runtime parameter editor or persistence.
- Global initialization is available through `AmclLocalizer` but has no desktop
  hotkey or toolbar control.
- LiDAR origin is the AMR center. There is no sensor offset, 3D geometry,
  scan matching, dynamic-obstacle model, or localization persistence.
- The planner/controller deliberately continue to use configured map semantics,
  not the AMCL estimate.
- Interactive visual acceptance remains unconfirmed because the available
  Windows automation could not capture this SFML window.

## Next smallest step

Perform a human-observed desktop acceptance pass: place and rotate Start, verify
the uncertain particle cloud and estimate marker, move manually and by planned
path, edit an obstacle, use Ctrl+R, and confirm Inspector/visual refresh and
navigation behavior. Address only a reproduced runtime or presentation defect.

## Important decisions

- The first native implementation uses standard AMCL concepts without ROS or an
  external localization dependency.
- Pending frame increments are composed into one scan-to-scan differential
  motion update so motion uncertainty does not depend on frame segmentation.
- Adaptive KLD resampling is standard sequential sampling over the normalized
  CDF; the standalone systematic resampler remains available and tested for
  fixed-count low-variance resampling.
- Convergence depends only on belief covariance. Ground-truth error is displayed
  and asserted in simulation tests but never fed into inference.
