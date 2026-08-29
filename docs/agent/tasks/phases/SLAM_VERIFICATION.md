# Phase: SLAM Verification

## Normal suites

- `SlamOccupancyGridTests`
- `CorrelativeScanMatcherTests`
- `SlamFrontendTests`
- `SlamIntegrationTests`

Add all four to `mingw32-make test`. Keep `SlamStressTests` and
`SlamBenchmark` as explicit extended targets.

## Metrics

Evaluation code exists only under `tests/`. Transform truth relative to the
initial truth pose before comparison with the SLAM-local frame.

- Position RMSE: square root of mean `dx^2 + dy^2`.
- Heading RMSE: square root of mean squared wrapped heading error.
- Occupied IoU: `TP / (TP + FP + FN)`.
- Occupancy agreement: correct known classifications divided by evaluated known
  classifications.
- Known coverage: classified cells divided by eligible evaluation cells.

Unknown must not count as free. Report missing/Lost trajectory samples rather
than dropping them from error metrics.

## Stress matrix

Use deterministic asymmetric maps and collision-free straight, turning,
corridor, and multi-turn paths. Cover ideal, nominal, and elevated odometry and
LiDAR noise with multiple fixed seeds. Include explicit no-feature and
large-motion safety cases. Calibrate gates from measured behavior, then record
per-case success, mean/worst RMSE, IoU, agreement, and coverage.

## Benchmark

Use prebuilt data, warmups, repeated `steady_clock` samples, finite-result and
candidate-bound assertions, and a checksum. Report median and p95 for one
91-beam 360-degree map integration, full match, and frontend update. Timing is
evidence, not a hardware-sensitive functional assertion.

## Final reviews and proof

Review architecture, algorithm correctness, and coverage after implementation.
Fix important findings, then rerun every final command in `NEXT_PLAN.md`.

Static ground-truth isolation review must classify all matches for:

```text
AMR|MapData|Environment|Simulator|Inspector|LidarSimulator|
OdometrySimulator|groundTruth|getAmrPose|truth
```

No match may occur in a production SLAM inference dependency.
