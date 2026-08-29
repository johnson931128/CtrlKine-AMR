Status: Approved
Last Updated: 2026-08-28
# SLAM V1 Specification

## 1. Scope

SLAM V1 estimates a 2D base pose and a local occupancy map using only
`OdometryDelta` and `LaserScan`. It provides deterministic log-odds mapping,
bounded correlative scan matching, and a frontend lifecycle.

Loop closure, pose graphs, graph optimization, ICP, map persistence, dynamic
obstacles, navigation on the SLAM estimate, and controller changes are outside
this specification.

## 2. Architectural requirements

### SLAM-ARCH-001 Sensor-only inference

Production SLAM inference shall accept only sensor abstractions and SLAM-owned
configuration/state. It shall not access `AMR`, `MapData`, simulator truth,
rendering, Inspector state, sensor simulators, or absolute odometry pose.

### SLAM-ARCH-002 Independent map ownership

SLAM shall own its occupancy grid. `MapData` remains simulator ground truth and
shall not be reused as, copied into, or queried by the SLAM estimate or matcher.

### SLAM-ARCH-003 Local frame

The first informative scan shall establish base pose `(0,0,0)` in the SLAM
frame. Truth may transform this local frame only for test metrics and display.

### SLAM-ARCH-004 Independent consumers

AMCL and SLAM shall receive the same immutable sensor measurements where their
update schedules overlap. Their reset, map, lifecycle, and inference state shall
remain independent.

## 3. Occupancy grid

### SLAM-MAP-001 Configuration

The grid shall have positive resolution and dimensions, a finite origin, finite
log-odds increments and clamps, ordered clamps, and classification thresholds
within the clamps. Invalid configuration shall be rejected.

### SLAM-MAP-002 State

Each cell shall store clamped log odds and whether it has been observed.
Unobserved cells are `Unknown`; observed cells at or below the free threshold
are `Free`; observed cells at or above the occupied threshold are `Occupied`;
observed values between thresholds are `Unknown`.

### SLAM-MAP-003 Coordinates

World-to-cell conversion shall use floor semantics relative to the configured
origin. The grid uses half-open bounds.

### SLAM-MAP-004 Inverse sensor model

For a physical hit (`finite`, `minRange <= range < maxRange`), traversed cells
before the endpoint receive free evidence and the endpoint receives occupied
evidence. For a finite range at or above `maxRange`, traversal is limited to
`maxRange`, cells receive free evidence, and no occupied endpoint is added.

Invalid ranges and invalid scan metadata shall not modify the grid. Sensor
x/y/yaw extrinsics and beam metadata shall be applied exactly once.

### SLAM-MAP-005 Determinism and commits

Ray traversal, clipping, corner behavior, and per-scan evidence resolution shall
be deterministic. Occupied evidence wins over free evidence for the same cell
within one scan. One successful scan integration increments the map revision
once; a rejected or non-integrated scan does not.

## 4. Scan matching

### SLAM-MATCH-001 Prediction

Odometry prediction shall compose `rotation1`, signed `translation`, and
`rotation2` using the existing y-down pose convention and normalize heading.

### SLAM-MATCH-002 Search

The matcher shall search configured bounded x/y/yaw offsets around the predicted
pose. It may use coarse-to-fine refinement. Candidate enumeration and tie
breaking shall be deterministic and bounded by diagnostics.

### SLAM-MATCH-003 Scoring

Only physical-hit beams may score occupied alignment. Full-circle beam selection
shall be cyclic and bounded. Unknown and out-of-bounds endpoints contribute no
positive evidence. Scoring shall not mutate the map.

### SLAM-MATCH-004 Acceptance and diagnostics

Acceptance requires configured geometric support and normalized score.
`ScanMatchResult` shall expose predicted/corrected poses, reason, score, selected
and used beams, candidate counts, and correction.

## 5. Frontend lifecycle

### SLAM-FRONT-001 Bootstrap

`Uninitialized` plus a structurally valid scan with physical hits shall
bootstrap at identity, integrate the scan, and enter `Tracking`. Invalid and
all-max first scans shall leave the frontend `Uninitialized` and the map
unchanged.

### SLAM-FRONT-002 Tracking

Tracking shall predict from valid odometry, match an informative scan, and
integrate only at an accepted corrected pose. Invalid/all-max scans use
prediction only and do not integrate. Poor matches do not integrate and count
as failures.

### SLAM-FRONT-003 Lost

Excessive odometry enters `Lost` immediately. Consecutive failed observations
enter `Lost` at a configured limit. Lost keeps the map frozen; an accepted
informative match may reacquire `Tracking`.

### SLAM-FRONT-004 Reset

Reset shall deterministically clear the local pose, lifecycle, diagnostics,
failure counters, and SLAM-owned map without affecting AMCL or `MapData`.

## 6. Simulator and UI

### SLAM-SIM-001 Sensor fan-out

Sensor acquisition shall be independent of consumer inference RNG. Odometry,
LiDAR, and AMCL shall have separate deterministic RNG streams. A generated scan
shall be reused rather than regenerated for each consumer.

### SLAM-SIM-002 Presentation

SLAM visualization and Inspector diagnostics shall be read-only. Estimated map,
corrected pose, predicted pose, and scan-match diagnostics shall be visible.
Truth-derived overlay transforms and error metrics shall be labeled display-only.

## 7. Verification

Deterministic tests shall cover mapping, matching, frontend lifecycle, simulator
integration, multiple trajectories, multiple sensor-noise cases and fixed seeds,
position and heading RMSE, occupied IoU, occupancy agreement, coverage, and
bounded scan-matching performance. Existing normal regression and localization
stress shall remain passing.
