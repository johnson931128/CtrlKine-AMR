# 360° LiDAR / Localization Work Package

## Mission

Convert the simulated LiDAR to clean full-circle semantics while preserving AMCL behavior and preparing a stable sensor abstraction for future SLAM.

Do not implement SLAM.

---

## 1. Explorer C Reading Scope

Read primarily:

```
include/LidarSimulator.hpp
src/LidarSimulator.cpp

include/LocalizationTypes.hpp

include/AmclLocalizer.hpp
src/AmclLocalizer.cpp

include/ParticleFilter.hpp
src/ParticleFilter.cpp

include/LocalizationVisualization.hpp
src/LocalizationVisualization.cpp

include/LocalizationConfig.hpp
src/LocalizationConfig.cpp
```

Relevant tests:

```
tests/LocalizationSensorTests.cpp
tests/ParticleFilterTests.cpp
tests/LocalizationIntegrationTests.cpp
tests/LocalizationStressTests.cpp
tests/LocalizationConfigTests.cpp
```

Do not deeply audit unrelated navigation code.

---

## 2. Audit Questions

Determine:

```
current FOV
current beam count
angleMin semantics
angleIncrement semantics
beam indexing
first/last beam behavior
range semantics
sensor extrinsics
AMCL beam assumptions
beam-skipping assumptions
rendering assumptions
configuration assumptions
future SLAM constraints
```

Return findings before production implementation.

---

## 3. Full-Circle Contract

Default LiDAR becomes:

```
fieldOfView = 2π
```

For `N` beams:

```
angle(i) = angleMin + i * (2π / N)

i = 0 ... N - 1
```

Do not generate both physical directions:

```
-π
+π
```

as separate duplicate rays.

The scan contains `N` unique directions.

---

## 4. Beam Seam

Explicitly test the full-circle seam.

For a simple low beam count, physical directions must be unique.

For example, a 4-beam full scan conceptually covers:

```
0°
90°
180°
270°
```

or an equivalent rotated origin depending on `angleMin`.

It must not contain:

```
0°
...
360°
```

as two separate beams.

---

## 5. LaserScan Data Model

Review and preserve useful fields:

```
ranges
angleMin
angleIncrement
minRange
maxRange
sensorOffsetX
sensorOffsetY
sensorYawOffset
```

Consider future timing metadata only if it has a clear semantic:

```
scanTime
timeIncrement
```

Do not implement rolling-scan motion distortion.

Do not add speculative fields without use.

---

## 6. Sensor Extrinsics

Full-circle semantics must continue respecting:

```
offsetX
offsetY
yawOffset
```

Simulation, AMCL prediction, and visualization must use consistent transforms.

An invalid sensor origin must fail safely according to existing scan conventions.

---

## 7. Beam Generation Tests

Add deterministic tests for:

```
beam count
360° angular coverage
angle increment
first beam
last beam
unique seam
heading rotation
yaw offset
x/y sensor offset
```

Use small beam counts such as:

```
4
8
```

to make geometry easy to verify.

---

## 8. Ray-Casting Tests

Use known geometry around the robot.

Cover:

```
north
south
east
west
diagonal
world boundary
sensor near boundary
translated sensor
rotated sensor
```

Verify ranges and validity semantics.

---

## 9. AMCL Compatibility

Do not allow the localization stack to assume:

```
front-facing LiDAR
270° FOV
fixed angleMin
specific starting beam
```

Verify:

```
beam selection
beam skipping
likelihood scoring
invalid range accounting
max-range accounting
sensor extrinsics
```

with full-circle scans.

---

## 10. Beam Selection vs Raw Scan

Preserve the distinction:

```
raw scan beam count
!=
AMCL selected beam count
!=
rendered beam count
```

Example:

```
raw simulated LiDAR
    360° / many beams

AMCL
    selects bounded subset

UI
    renders another bounded subset
```

Do not increase inference cost unnecessarily just because scan FOV increased.

---

## 11. Visualization

F2/F3 semantics must remain usable.

Requirements:

```
LiDAR rays toggle preserved
hit-point toggle preserved
360° visual coverage
render subsampling independent of inference
no seam duplication
extrinsics visibly respected
```

Do not render hundreds of rays by default.

---

## 12. AMCL Regression

Verify:

```
local initialization
global localization
tracking
ambiguity
convergence
recovery
kidnapped robot
open-map false-convergence protection
KLD
beam skipping
configuration loading
```

Do not weaken confidence thresholds merely to restore a failing test.

---

## 13. Stress Tests

Existing reference:

```
feature-rich local:
10/10 seeds

global:
9/10 within documented acquisition bound

open map:
0/10 false convergence

kidnapped recovery:
5/5
```

Record new results after 360° LiDAR.

Improvement is welcome but not required.

Do not rewrite expectations solely to claim improvement.

---

## 14. Performance

Measure representative:

```
LiDAR simulation
sensor weighting
clustering
KLD
```

Remember that increasing raw scan coverage does not require AMCL to consume every beam.

Do not optimize without measurements.

---

## 15. Future SLAM Requirement

After this package, a future SLAM module should conceptually receive:

```
Odometry measurement
LaserScan
```

without accessing:

```
AMR ground-truth pose
Simulator internals
Inspector state
```

The scan abstraction should be reusable by AMCL and future SLAM.

Do not implement map building.

---

## 16. Explorer C Deliverable

Return:

1. current scan contract,
2. problems with current FOV/angle semantics,
3. AMCL assumptions,
4. rendering assumptions,
5. proposed 360° contract,
6. data-model changes if required,
7. future-SLAM boundary,
8. test recommendations,
9. performance risks.

Initial Explorer work is read-only.

---

## 17. Sensor Reviewer

After implementation inspect:

```
full-circle math
seam handling
ray casting
extrinsics
AMCL compatibility
beam skipping
test coverage
stress coverage
performance
future SLAM boundary
```

Reject accidental coupling to ground truth.

