# Project Status

## Current milestone

AMCL hardening, observability, localization workbench, and the optional
localization-driven navigation bridge are implemented. The established
simulation-truth navigation mode remains the default.

## Completed behavior

- `AMR`, `OdometrySimulator`, `LidarSimulator`, `MapLikelihoodField`,
  `ParticleFilter`, and `AmclLocalizer` retain separate truth, measurement,
  derived-map, belief, and lifecycle ownership. Inference never receives truth
  as an estimate, and rendering never consumes inference RNG.
- The likelihood representation now performs exact continuous Euclidean
  distance queries against cached obstacle-cell AABBs and the world boundary.
  Geometry revision/signature changes invalidate it; free/occupied point
  semantics also prevent particles inside mapped geometry from earning sensor
  likelihood.
- Sensor Model V2 uses finite log-likelihood accumulation, normalized hit/random
  terms, deterministic beam selection, explicit invalid/max-range accounting,
  and particle-fraction beam skipping with safe majority-corruption fallback.
  LiDAR x/y/yaw extrinsics are shared by simulation, prediction, and drawing;
  an out-of-map extrinsic origin yields a safe invalid scan rather than a crash.
- Particle clustering uses deterministic neighboring x/y/yaw bins with yaw
  wraparound and a relative bin-mass connectivity floor, so negligible recovery
  particles cannot bridge supported modes. Each cluster exposes count, weight,
  circular pose, covariance, spatial extent, heading extent, and resultant.
- The reported pose/covariance comes from a hysteresis-stabilized dominant
  cluster. Whole-filter diagnostics remain separate. Comparable modes report
  `Ambiguous`; diffuse beliefs report `Tracking`; mismatch reports `Recovering`.
- Localization support is `Insufficient` with no obstacles, `Weak` without a
  usable/informative measurement or with significant/repeated hypotheses, and
  `Good` only with supported single-mode evidence. All-max scans propagate
  pending motion without creating sensor evidence. Global convergence requires
  25 sensor updates and does not suppress ESS/recovery resampling.
- KLD sampling was checked against exact reference counts, bounds, `pfErr`, and
  `pfZ`. Recovery requires disabled zero/zero alphas or `alphaFast > alphaSlow`,
  rises on severe mismatch, injects free-space hypotheses, and decays after
  reconvergence. A fixed-capacity localization history records belief, odometry,
  ESS, cluster, uncertainty, recovery, and observation-quality metrics.
- `localization.cfg` persists validated AMCL, motion, likelihood, clustering,
  convergence, beam-skip, LiDAR/extrinsic, navigation-gate, and seed values.
  Loading is atomic; unknown, malformed, non-finite, or inconsistent values
  retain the previous/default configuration and cannot crash the application.
- Desktop controls: F1 particles, F2 LiDAR rays, F3 hit points, F4 AMCL
  estimate, F6 covariance, F7 odometry, F8 diagnostics, Ctrl+G global
  localization, Ctrl+L localization-only reset, Ctrl+M navigation mode, and
  guarded Ctrl+Shift+K kidnap-at-cursor. Ctrl+R remains robot reset. Failed
  localization initialization clears stale scan/particle presentation.
- Default rendering keeps LiDAR rays off, renders particles subtly, draws
  state-dependent AMCL markers, a true covariance eigen-ellipse, and optional
  odometry. A compact always-visible shape/text legend identifies Robot, Start,
  Goal, Path, Particles, AMCL, Odometry, and LiDAR. Inspector reports state,
  support, initialization, ESS, entropy, clusters, recovery, estimate/truth/
  odometry comparison, covariance, beam accounting, quality/contrast, layers,
  planning source, and stop reason.
- Global localization spreads only the particle belief. Local reset does not
  teleport truth. Kidnap validates the full AMR footprint, teleports only truth,
  rebases the ground-truth odometry reference without teleporting odometry, and
  lets recovery occur from new scans.
- `PathPlanner` has an additive explicit-start/goal API that does not mutate
  persistent Start/Goal. Optional localization-driven navigation plans from the
  trusted dominant estimate, gates on state/support/dominance/covariance, uses
  that estimate for waypoint commands while truth executes/collides, and stops
  with explicit replan required if confidence is lost.

## Verification

- Normal clean build/regression: 164 PASS, 0 FAIL across ten executables:
  CoordinateMapper 6, MapData 16, MapDataFile 15, PathPlanner 16,
  PathExecution 19, SimulatorRuntime 15, LocalizationSensor 26,
  ParticleFilter 35, LocalizationIntegration 10, LocalizationConfig 6.
- Extended deterministic stress: 4 PASS, 0 FAIL. Feature-rich local localization
  succeeded 10/10 seeds (mean final position error 11.0719); global localization
  converged within the documented 125-unit/1.25-rad acquisition bound on 9/10
  seeds with zero transient false-convergence updates; open maps produced 0/10
  false convergence; kidnapped recovery succeeded 5/5 seeds.
- Regression scenarios cover no obstacles, repeated geometry and ambiguity,
  symmetry-breaking evidence, sparse/feature-rich tracking, corridor covariance
  anisotropy, boundaries/extrinsics, high odometry noise, LiDAR outliers and beam
  skipping, map edits, lifecycle resets, global initialization, recovery, RNG
  independence, explicit planning, navigation gating, and confidence-loss stop.
- Final representative benchmark (milliseconds): field rebuild 0.0098, 91-beam
  LiDAR 0.1397. At 300/1000/2000 particles with 31 beams, sensor weighting was
  3.379/11.177/22.878, clustering 0.514/1.806/3.868, and KLD resampling
  0.114/0.386/0.887. At 5000 particles and 31 beams: sensor 55.799,
  clustering 8.318, KLD 2.308. At 2000 particles and 91 beams: sensor 62.058.
- The desktop executable launched, stayed alive, and initially exposed the
  expected titled window. The automation provider then failed to enumerate the
  SFML window (`0x80070003`), and direct capture lost the window handle, so no
  visual or hotkey behavior is claimed as human-observed.

## Known limitations

- Localization support is an interpretable first-version heuristic, not a
  formal observability proof. Repeated-geometry detection is map-level and
  conservative.
- The exact continuous distance query is linear in obstacle AABBs; measured
  sensor weighting is the dominant cost at high particle/beam counts.
- The symmetric regression proves ambiguity retention under repeated geometry,
  then convergence after a map-supported, higher-information full scan and
  motion. It does not claim every symmetry can be broken by motion alone.
- Global stress is 9/10 at the documented coarse acquisition bound; one seed
  remains safely `Tracking` with unusable current measurement rather than
  falsely reporting convergence.
- Desktop visual acceptance and live hotkey interaction remain unverified in
  this automation environment. The localization file has no in-app parameter
  editor, and localization-driven navigation intentionally requires explicit
  replan after confidence loss.

## Next smallest meaningful milestone

Perform a human desktop acceptance pass for the legend/layers, no-obstacle
diagnostics, global/local reset distinction, kidnap recovery, and both
navigation modes; then investigate the one bounded global-stress seed that
remains `Tracking` without weakening the false-convergence gate.

## Important decisions

- Ground-truth error is display/test-only and never influences runtime state,
  support, confidence, recovery, resampling, planning gate, or control.
- Max-range beams are explicitly counted but treated as uninformative by the
  likelihood-field endpoint model; visualization subsampling is independent of
  the raw scan used for inference.
- Multi-modal beliefs are never presented through the unsupported whole-filter
  mean. Default-off localization navigation preserves the stable original mode.
