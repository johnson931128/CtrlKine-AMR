# Verification Work Package

## Mission

Verify that the architecture/UI/LiDAR refactor preserved the working navigation and localization baseline.

Do not treat compilation alone as completion.

---

## 1. Baseline Before Refactor

Before production changes run:

```
mingw32-make clean
mingw32-make all
mingw32-make test
mingw32-make test-localization-stress
```

Expected historical reference:

```
normal:
164 PASS
0 FAIL

stress:
4 PASS
0 FAIL
```

If baseline fails unexpectedly, investigate before large refactoring.

---

## 2. Incremental Verification

After each structural extraction:

```
build
targeted tests
```

Examples:

```
UI extraction
    → runtime/UI-focused tests

localization renderer extraction
    → localization sensor/runtime tests

LiDAR semantics
    → LocalizationSensorTests
```

Do not accumulate many unverified changes.

---

## 3. Navigation Regression

Verify existing behavior:

```
Start placement
Start rotation
Goal placement
planning
clearance-aware planning
path execution
Ctrl+R
truth navigation
AMCL navigation
confidence-loss stop
```

Route semantics must remain unchanged.

---

## 4. Localization Regression

Verify:

```
local initialization
global initialization
tracking
ambiguity
convergence
recovery
kidnap
beam skipping
KLD
sensor extrinsics
configuration loading
no-obstacle false-convergence protection
```

Ground truth must never become estimator input.

---

## 5. UI Logic Regression

Test where practical:

```
default Inspector tab
tab switching
tab persistence
scroll behavior
resize/layout calculations
debug layer toggle state
navigation mode toggle
global localization command
localization-only reset
robot reset distinction
```

Do not test SFML pixels in ordinary unit tests.

---

## 6. 360° LiDAR Regression

Explicit tests must cover:

```
full-circle coverage
no duplicate seam
beam count
angle increment
sensor heading
sensor yaw offset
sensor translation
four cardinal directions
boundary interaction
AMCL compatibility
visual subsampling semantics
```

---

## 7. Extended Localization Stress

Run:

```
mingw32-make test-localization-stress
```

Compare with previous behavior.

Track:

```
local localization seed success
global localization seed success
false convergence
kidnapped recovery
```

Never improve metrics by loosening false-convergence protection.

---

## 8. Benchmark

Run if supported:

```
mingw32-make localization-benchmark
```

Record representative:

```
field/derived-map work
LiDAR simulation
sensor weighting
clustering
KLD resampling
```

Investigate major unexplained regressions.

Do not optimize merely because a number changed slightly.

---

## 9. Clean Final Regression

At the end:

```
mingw32-make clean
mingw32-make all
mingw32-make test
```

Then run every produced normal test executable directly if practical.

Then:

```
mingw32-make test-localization-stress
mingw32-make localization-benchmark
```

where supported.

Required final target:

```
0 FAIL
```

---

## 10. Architecture Reviewer

Review final code for:

```
Simulator responsibility
UI ownership
Environment responsibility
localization ownership
sensor ownership
state ownership
future SLAM coupling
```

Resolve high-confidence architectural regressions.

---

## 11. UI Reviewer

Review:

```
tab grouping
duplicated information
hard-coded layout
scroll behavior
resize behavior
visual hierarchy
toolbar clutter
legend placement
localization overlay clarity
LiDAR visibility
```

Resolve concrete issues.

---

## 12. Sensor / Test Reviewer

Review:

```
360° math
seam semantics
extrinsics
AMCL assumptions
new tests
existing localization tests
stress tests
test weakening
ground-truth leakage
```

Do not accept tests that pass because inference gained access to truth.

---

## 13. Desktop Acceptance Attempt

Launch the real application.

Attempt to verify:

```
larger default window
reasonable Inspector width
Map tab
Navigation tab
Localization tab
tab switching
Inspector scrolling
map zoom
toolbar
legend
particle cloud
AMCL estimate
covariance
odometry
360° LiDAR
LiDAR layer toggles
global localization
local reset
robot reset
kidnap recovery
truth navigation
AMCL navigation
```

If SFML capture/automation fails:

```
do not claim visual verification
```

Instead record:

```
application launched
process/window survival
automation limitation
human acceptance required
```

---

## 14. Human Acceptance Checklist for Final Report

Provide a concise sequence such as:

```
1. Launch application.
2. Resize window.
3. Switch all Inspector tabs.
4. Verify Inspector scroll.
5. Place Start / Goal / obstacles.
6. Run truth navigation.
7. Toggle localization layers.
8. Enable LiDAR rays and confirm full-circle coverage.
9. Run global localization.
10. Test localization-only reset.
11. Test robot reset.
12. Test kidnap recovery.
13. Switch to localization-driven navigation.
14. Confirm confidence-loss behavior if reproducible.
```

Do not add a large tutorial to the application.

---

## 15. STATUS Verification Record

Update:

```
docs/agent/STATUS.md
```

Include:

```
normal PASS/FAIL total
stress results
benchmark
desktop verification status
human verification still required
known limitations
```

---

## 16. Completion Condition

Verification is complete only if:

```
build passes
normal regression passes
stress suite passes
navigation preserved
AMCL preserved
360° tests pass
reviewers find no unresolved critical issue
STATUS updated
```

Final target:

```
0 FAIL
```

