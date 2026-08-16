# NEXT_PLAN.md

# Milestone: Pre-SLAM Architecture, UI, and 360° LiDAR Refactor

## 0. Mission

Perform a deliberate architecture and presentation refactor before beginning SLAM work.

The repository has reached a point where the major subsystems are functionally meaningful:

* Map editing
* Map persistence
* Start / Goal configuration
* A* path planning
* Clearance-aware navigation
* Path execution
* AMR runtime motion
* Simulated odometry
* Simulated LiDAR
* AMCL particle filtering
* Likelihood-field localization
* KLD adaptive sampling
* Recovery
* Particle clustering
* Ambiguity detection
* Global localization
* Kidnapped-robot recovery
* Optional localization-driven navigation
* Localization visualization and diagnostics

The next milestone must **not** add another major algorithm.

Instead, consolidate the codebase so that future SLAM, navigation, and sensor work can be added without turning `Simulator`, `Environment`, or localization code into monolithic modules.

Primary goals:

1. Audit and improve subsystem responsibilities.
2. Reduce `Simulator` responsibility and coupling.
3. Reorganize the right-side Inspector into clean category tabs.
4. Improve general desktop visual polish and layout.
5. Convert the LiDAR model to clean 360° full-circle semantics.
6. Prepare LiDAR data structures for future SLAM without implementing SLAM.
7. Preserve all existing navigation and AMCL behavior.
8. Preserve or improve testability.
9. Perform a final architecture review after implementation.

This is intentionally a large multi-phase task.

Do not stop after the first refactor or first successful build.

---

# 1. Current Baseline

Current repository state includes the hardened AMCL milestone.

Known normal regression baseline:

```text
164 PASS
0 FAIL
```

Known extended localization stress baseline:

```text
4 PASS
0 FAIL
```

Current relevant capabilities include:

```text
AMR
MapData
Environment
MapValidator
PathPlanner
PathExecution

LidarSimulator
OdometrySimulator
MapLikelihoodField
ParticleFilter
AmclLocalizer

LocalizationConfig
LocalizationVisualization

Simulator
```

The stable simulation-truth navigation mode remains the default.

Localization-driven navigation is optional.

Ground truth must remain separate from localization inference.

---

# 2. User-Observed Runtime Issues

The current desktop runtime is functionally strong but visually and architecturally rough.

Observed issues:

## 2.1 Inspector overload

The right-side Inspector contains too many unrelated sections in one long vertical stream.

Examples currently include:

```text
Cursor
Map Stats
Map Validation
Selected Object
Robot State
Path Planning
Navigation
Localization
Sensor diagnostics
Particle diagnostics
Recovery
Layers
etc.
```

The problem is not that the information is unnecessary.

The problem is lack of information hierarchy.

The Inspector should become a category-based panel similar to a browser or IDE tab layout.

Target conceptual organization:

```text
Map | Navigation | Localization
```

Potentially a fourth category is allowed if architecture review clearly justifies it.

Do not create tabs solely for visual symmetry.

## 2.2 Window and layout density

The current application window and panel proportions are becoming too small for the amount of information displayed.

Review:

* default window size,
* minimum practical window size,
* simulation viewport size,
* Inspector width,
* toolbar density,
* legend placement,
* text spacing,
* scroll behavior,
* resize behavior.

The UI should feel less cramped while preserving usability on ordinary desktop resolutions.

## 2.3 Visual roughness

Some visual elements are technically correct but do not feel polished or visually coherent.

Review:

* spacing,
* alignment,
* typography hierarchy,
* marker scale,
* alpha/transparency,
* status colors,
* tab highlighting,
* selection outlines,
* debug overlays,
* localization markers,
* covariance rendering,
* LiDAR rendering,
* legend presentation.

Do not redesign the application into a different product.

Keep the existing lightweight SFML style.

## 2.4 LiDAR field of view

The current LiDAR default is not a full-circle scan.

For future 2D SLAM work, the simulator should support clean 360° planar LiDAR semantics.

Do not implement SLAM in this milestone.

---

# 3. High-Level Strategy

Use multi-agent reasoning primarily for:

```text
parallel architecture analysis
parallel responsibility audit
parallel review
```

Do not let several agents independently rewrite overlapping production files.

Recommended structure:

```text
Main Agent
    architecture owner
    integration owner
    production implementation owner

Explorer A
    architecture / responsibility audit

Explorer B
    UI / rendering / Inspector audit

Explorer C
    sensor / localization / future-SLAM audit
```

After implementation:

```text
Reviewer A
    architecture review

Reviewer B
    runtime / UI review

Reviewer C
    regression / test review
```

If agent capacity is unavailable:

* continue serially,
* do not repeatedly retry failed spawns,
* do not stop the task.

---

# 4. Required Initial Reading

Before modifying production code, read:

```text
AGENTS.md
docs/agent/STATUS.md
docs/specs/SystemOverview.md
docs/specs/MapCoordinateSpec.md
docs/specs/PathPlannerSpec.md
```

Then inspect at minimum:

```text
include/Simulator.hpp
src/Simulator.cpp

include/Environment.hpp
src/Environment.cpp

include/AMR.hpp
src/AMR.cpp

include/MapData.hpp
src/MapData.cpp

include/PathPlanner.hpp
src/PathPlanner.cpp

include/PathExecution.hpp
src/PathExecution.cpp

include/LidarSimulator.hpp
src/LidarSimulator.cpp

include/LocalizationTypes.hpp

include/LocalizationVisualization.hpp
src/LocalizationVisualization.cpp

include/AmclLocalizer.hpp
src/AmclLocalizer.cpp

include/ParticleFilter.hpp
src/ParticleFilter.cpp

include/LocalizationConfig.hpp
src/LocalizationConfig.cpp

Makefile
```

Inspect all current test executables relevant to these modules.

Do not perform broad unrelated repository reading.

---

# 5. Phase 1 — Baseline Verification

Run:

```text
mingw32-make clean
mingw32-make all
mingw32-make test
```

Run all normal test executables directly.

Also run:

```text
mingw32-make test-localization-stress
```

if still supported by the current Makefile.

Expected baseline:

```text
Normal:
164 PASS
0 FAIL

Extended localization stress:
4 PASS
0 FAIL
```

If baseline differs, investigate before starting architecture changes.

Do not begin a large refactor on top of unexplained failures.

---

# 6. Phase 2 — Architecture Audit

Use Explorer A.

Perform a responsibility and dependency audit of the current codebase.

For every major class, identify:

```text
What state does it own?
What behavior does it own?
What other modules does it know about?
What other modules know about it?
Does it perform rendering?
Does it perform input handling?
Does it perform algorithmic work?
Does it perform persistence?
Does it coordinate other modules?
```

Pay particular attention to:

```text
Simulator
Environment
AMR
MapData
PathPlanner
PathExecution
AmclLocalizer
ParticleFilter
LocalizationVisualization
```

Identify:

* god-object tendencies,
* duplicated state,
* cyclic conceptual dependencies,
* inappropriate rendering dependencies,
* inappropriate algorithm/UI dependencies,
* helper functions living in the wrong module,
* stale or misleading abstractions,
* structures that mix persistent and runtime state,
* places where future SLAM would worsen coupling.

Do not edit production code during this audit.

Produce a concise architecture recommendation for the Main Agent.

---

# 7. Phase 3 — UI / Rendering Audit

Use Explorer B.

Inspect:

```text
window layout
toolbar
Inspector
legend
scrolling
selection UI
localization overlays
LiDAR drawing
path drawing
map drawing
robot drawing
```

Determine which UI responsibilities currently belong to `Simulator`.

Identify candidates that could reasonably move into dedicated components.

Possible conceptual components include:

```text
InspectorPanel
Toolbar
LocalizationRenderer
SimulationViewport
DebugOverlay
```

These names are examples only.

Do not create classes merely to reduce line count.

A new class must own a coherent responsibility.

For the Inspector specifically, propose:

```text
tabs
sections per tab
layout behavior
scroll behavior
resize behavior
selection/state presentation
```

Do not edit production code during the audit.

---

# 8. Phase 4 — LiDAR / Future SLAM Audit

Use Explorer C.

Inspect:

```text
LidarSimulator
LaserScan
LocalizationTypes
AMCL sensor update path
LiDAR visualization
LocalizationConfig
```

Determine:

* current FOV semantics,
* beam indexing semantics,
* angle representation,
* whether first/last beam duplicate for a full-circle scan,
* range semantics,
* extrinsics semantics,
* current scan metadata,
* assumptions inside AMCL,
* assumptions inside rendering,
* assumptions future SLAM would need removed.

Design a clean full-circle scan contract.

Future SLAM compatibility should be considered, but SLAM itself must remain out of scope.

Do not edit production code during the audit.

---

# 9. Phase 5 — Main Architecture Decision

After all audits:

The Main Agent must independently verify important claims in source.

Then create an internal target responsibility map.

A healthy target should roughly preserve:

```text
MapData
    persistent map/domain state

Environment
    editor-facing map interactions and environment rendering

AMR
    robot ground-truth runtime state and body geometry

PathPlanner
    planning algorithm

PathExecution
    path execution state

LidarSimulator
    LiDAR measurement simulation

OdometrySimulator
    odometry measurement simulation

MapLikelihoodField
    derived localization map representation

ParticleFilter
    particle belief algorithm

AmclLocalizer
    localization lifecycle/orchestration

Simulator
    application-level orchestration

UI components
    presentation and interaction only
```

`Simulator` may coordinate subsystems.

It should not contain large amounts of:

```text
Inspector formatting
tab rendering
localization drawing math
generic toolbar layout logic
algorithmic localization calculations
```

Do not force excessive abstraction.

The goal is clearer responsibility, not maximum class count.

---

# 10. Phase 6 — Refactor Safety Rules

Architecture refactor must preserve observable behavior unless explicitly changed later in this plan.

During structural refactoring:

```text
no new navigation algorithm
no new localization algorithm
no SLAM
no path smoothing
no 8-neighbor A*
no controller redesign
```

After each major extraction:

```text
build
targeted tests
```

Do not perform the entire refactor and test only at the end.

---

# 11. Phase 7 — Simulator Responsibility Reduction

Inspect `Simulator.cpp` carefully.

Separate responsibilities where justified.

Likely extraction candidates include:

```text
Inspector rendering/state
Localization overlay rendering
Toolbar presentation
UI layout helpers
```

Do not move:

```text
application lifecycle orchestration
subsystem coordination
mode transitions
high-level event routing
```

out of `Simulator` unless there is a clearly better owner.

The Main Agent must avoid introducing an equally large replacement god object.

---

# 12. Phase 8 — Localization Rendering Responsibility

Review `LocalizationVisualization`.

If it is currently too small or too limited relative to the rendering responsibility still inside `Simulator`, evolve it into a coherent localization presentation module.

Potential responsibilities:

```text
particle cloud drawing
AMCL estimate marker
covariance ellipse
odometry marker
LiDAR hit-point visualization
LiDAR ray visualization
localization visual style
```

It must not:

```text
change particle weights
consume inference RNG
perform AMCL updates
change localization state
```

Rendering must remain observational.

---

# 13. Phase 9 — UI Layout Model

Improve the desktop layout.

Review the current default window dimensions.

Select a larger default size appropriate for the current application.

Target characteristics:

```text
larger default viewport
wider Inspector
comfortable toolbar spacing
clear tab strip
reasonable minimum usable dimensions
proper resizing
```

Do not hard-code assumptions that only work at one exact resolution.

Ensure simulation viewport and Inspector geometry update correctly on resize.

---

# 14. Phase 10 — Inspector Tab Architecture

Replace the single long Inspector information stream with tabs.

Minimum target tabs:

```text
Map
Navigation
Localization
```

Recommended content:

## Map

```text
Cursor
Map Stats
Map Validation
Selected Object
```

## Navigation

```text
Robot State
Navigation Mode
Planning Start Source
Path Planning
Path Execution
Stop Reason
Goal / Start information where relevant
```

## Localization

```text
Localization State
Localization Support
Initialization Mode
Estimated Pose
Odometry Pose
Ground Truth diagnostic
Position / Heading error diagnostic
Covariance
Particles
ESS
Entropy
Clusters
Dominant Weight
Second Weight
Recovery
Sensor quality
Beam accounting
Visualization layers
```

If information is duplicated across tabs, remove duplication where possible.

Do not create a fourth tab unless audit demonstrates a clear category.

---

# 15. Phase 11 — Inspector Interaction

Implement tab interaction.

Requirements:

```text
click tab to activate
active tab visibly distinct
only active tab body displayed
tab selection persists while application runs
scroll state behaves predictably
```

Preferred behavior:

Each tab may keep its own scroll offset.

If that substantially complicates the UI, a single reset-on-tab-switch scroll offset is acceptable, but behavior must be deterministic.

Mouse wheel over Inspector scrolls Inspector.

Mouse wheel over simulation viewport retains map zoom behavior.

No event leakage between regions.

---

# 16. Phase 12 — Inspector Information Hierarchy

Within each tab:

Use visual hierarchy.

Examples:

```text
primary state
secondary metrics
diagnostic details
```

Localization should not show dozens of values with equal visual importance.

Example hierarchy:

```text
Localization
State: Converged
Support: Good

Estimate
X
Y
Yaw

Confidence
Position σ
Heading σ
Dominant Weight

Particle Filter
Particles
ESS
Clusters

Sensor
Used beams
Skipped beams
Quality

Recovery
Probability
```

Use spacing and headings consistently.

Do not overdecorate.

---

# 17. Phase 13 — Inspector Formatting Helpers

Remove repeated ad-hoc `ostringstream` and layout logic from `Simulator` where sensible.

A small structured UI model is allowed.

Example conceptual model:

```text
InspectorSection
    title
    lines
    status/style
```

or equivalent.

Do not create a generic UI framework.

The implementation should remain understandable C++/SFML code.

---

# 18. Phase 14 — Toolbar Review

Review current toolbar controls.

Do not remove existing editor functionality.

Improve:

```text
spacing
active-state visibility
grouping
readability
resize behavior
```

If localization debug hotkeys are currently only discoverable through external knowledge, consider exposing concise hints in the Localization tab rather than adding many toolbar buttons.

Avoid toolbar overcrowding.

---

# 19. Phase 15 — Legend Review

Keep the legend, but improve placement and readability if needed.

It should clearly distinguish:

```text
Robot
Start
Goal
Path
Particles
AMCL Estimate
Odometry
LiDAR
```

Do not let the legend block meaningful map content.

A compact overlay or UI-region placement is preferred.

---

# 20. Phase 16 — 360° LiDAR Contract

Change the default LiDAR to full-circle scanning.

Target:

```text
fieldOfView = 2π
```

But do not implement it as a naive inclusive `[-π, +π]` sequence that duplicates the same physical ray.

For a full-circle scan with `N` beams:

```text
beam i
angle = angleMin + i * (2π / N)

i = 0 ... N - 1
```

The first physical direction must not be duplicated as the final beam.

Define explicit full-circle semantics in code.

Tests must verify this.

---

# 21. Phase 17 — LaserScan Data Model Review

Review the current `LaserScan` data structure.

Preserve useful fields:

```text
ranges
angleMin
angleIncrement
minRange
maxRange
sensor extrinsics
```

Prepare for future SLAM by considering scan timing metadata.

Allowed additions include:

```text
scanTime
timeIncrement
```

if the architecture audit finds them useful.

Do not simulate rolling-scan motion distortion yet.

Do not add timestamps unless they have a clear current or near-future semantic.

Avoid speculative data-model bloat.

---

# 22. Phase 18 — Full-Circle Beam Generation Tests

Add direct tests for:

```text
360° coverage
beam count
angle increment
first beam angle
last beam angle
no duplicate first/last direction
heading rotation
sensor yaw offset
sensor x/y offset
```

Test a small beam count such as:

```text
4 beams
8 beams
```

to make expected angles obvious.

Example four-beam geometry should represent four unique directions.

---

# 23. Phase 19 — 360° Ray-Casting Tests

Validate full-circle LiDAR against known map geometry.

Create deterministic tests covering:

```text
north obstacle
south obstacle
east obstacle
west obstacle
diagonal obstacle
world boundary
sensor near boundary
sensor offset
rotated sensor
```

All ranges must remain finite/valid according to existing scan semantics.

---

# 24. Phase 20 — AMCL 360° Compatibility

The AMCL sensor model must continue working with 360° scans.

Do not assume:

```text
front-facing scan
270° field of view
angleMin fixed to a particular value
```

Verify:

```text
beam selection
beam skipping
likelihood scoring
max-range accounting
invalid-range handling
LiDAR extrinsics
```

with the new full-circle scan.

Existing localization convergence tests must remain valid.

If deterministic parameter retuning is genuinely needed because 360° adds information, make the smallest justified changes.

Do not weaken confidence gates.

---

# 25. Phase 21 — LiDAR Visualization with 360° Data

LiDAR visualization must remain readable with a full-circle scan.

Remember:

```text
sensor beam count
!=
rendered beam count
```

Inference may use many beams.

Rendering may display a subset.

Requirements:

```text
F2 LiDAR ray toggle preserved
F3 hit point toggle preserved
render subsampling independent of inference
no duplicate visual ray at seam
sensor extrinsics reflected visually
```

Do not default to rendering hundreds of rays.

---

# 26. Phase 22 — Visual Polish Pass

Perform a deliberate desktop visual pass.

Review:

```text
background
grid contrast
obstacle contrast
AMR contrast
Start / Goal markers
path
particles
estimate
covariance
odometry
LiDAR hits
selection outlines
legend
toolbar
Inspector
tabs
```

Fix small obvious issues such as:

```text
awkward spacing
overlapping labels
too-thick outlines
hard-to-read low contrast
unbalanced padding
misaligned text
markers visually overwhelming nearby content
```

Keep style understated.

Do not spend large effort on cosmetic animation.

---

# 27. Phase 23 — Rendering Smoothness Audit

Inspect whether visible roughness comes from:

```text
frame-rate-dependent animation
unnecessary geometry rebuild
integer rounding
view transforms
abrupt visual state changes
path marker placement
particle rendering
```

Fix clear presentation defects.

Do not redesign the navigation controller under this task.

If motion itself appears mechanically stop-turn-go, leave controller behavior unchanged and record it as a separate future navigation milestone.

---

# 28. Phase 24 — Input and UI Responsibility Audit

After UI refactor, inspect event handling again.

Ensure `Simulator` event routing remains understandable.

Avoid one giant event function containing all Inspector/tab logic.

Consider small UI-level handlers when coherent.

Examples:

```text
handleInspectorClick()
handleToolbarClick()
handleLocalizationHotkeys()
```

Do not over-fragment trivial code.

---

# 29. Phase 25 — State Ownership Audit

Verify after refactor that there is exactly one authoritative owner for important state.

Examples:

```text
MapData
    map state

AMR
    ground-truth runtime pose

PathExecution
    path execution progress

AmclLocalizer
    localization state

Inspector/UI
    active tab / UI presentation state
```

Do not let UI components own algorithm state.

Do not let rendering mutate domain state.

---

# 30. Phase 26 — Include Dependency Audit

Review headers.

Reduce unnecessary transitive includes where practical.

Use forward declarations where they materially reduce coupling and remain readable.

Do not perform a massive include-cleanup campaign unrelated to the refactor.

Check for headers that expose implementation-only SFML or localization dependencies unnecessarily.

---

# 31. Phase 27 — File Organization Review

The project currently broadly separates:

```text
include/
src/
tests/
```

Do not automatically create many nested directories.

First determine whether responsibility clarity is sufficiently improved through class boundaries alone.

Only introduce subdirectories if they materially improve navigation.

Potential categories could eventually be:

```text
navigation/
localization/
ui/
simulation/
```

but this milestone must not reorganize the entire repository merely for aesthetics.

A large path move creates merge/history noise.

Prefer responsibility refactoring over directory churn.

If no directory restructuring is necessary, explicitly keep the current top-level layout.

---

# 32. Phase 28 — Regression Tests for UI State Logic

Rendering pixels do not need exhaustive tests.

But non-rendering UI logic should be testable where practical.

Test:

```text
default active Inspector tab
tab switching
tab persistence
layer toggle state
resize layout calculations where accessible
global/local localization controls remain mapped correctly
navigation mode state preserved
```

Do not build a GUI testing framework.

---

# 33. Phase 29 — Architecture Regression Tests

Where extraction creates new pure logic, add tests.

Examples:

```text
Inspector content selection
covariance visualization math if moved
LiDAR angle generation
viewport layout calculations
```

Keep tests focused on logic, not SFML draw calls.

---

# 34. Phase 30 — Existing Navigation Regression

Re-run and verify:

```text
Start placement
Goal placement
A* planning
clearance-aware planning
path execution
Ctrl+R reset
truth navigation
localization-driven navigation
confidence-loss stop
```

The architecture/UI refactor must not alter route semantics.

---

# 35. Phase 31 — Existing Localization Regression

Re-run:

```text
local initialization
global initialization
tracking
ambiguity
convergence
recovery
kidnapped robot
no-obstacle false-convergence protection
beam skipping
KLD
sensor extrinsics
configuration loading
```

360° LiDAR must not compromise these behaviors.

---

# 36. Phase 32 — Extended Localization Stress

Run the extended stress suite.

Existing reference:

```text
feature-rich local localization:
10 / 10 seeds

global localization:
9 / 10 seeds within documented acquisition bound

open map:
0 / 10 false convergence

kidnapped recovery:
5 / 5 seeds
```

The refactor should not worsen false convergence.

If 360° LiDAR improves global localization, record the new result.

Do not change the test to force improvement.

---

# 37. Phase 33 — Performance Sanity

Repeat representative localization benchmark.

Previous approximate hotspots included sensor weighting.

Check that refactoring and 360° raw scan generation have not introduced major regressions.

Measure at least:

```text
LiDAR simulation
sensor weighting
clustering
KLD resampling
```

Remember that raw scan beam count and AMCL selected beam count can differ.

Do not optimize without evidence.

---

# 38. Phase 34 — Multi-Agent Architecture Review

After production implementation, use Reviewer A.

Review specifically:

```text
Simulator responsibility
Environment responsibility
UI ownership
rendering ownership
localization ownership
sensor ownership
future SLAM extension points
```

Ask:

```text
Would adding a SLAM subsystem now force major unrelated changes?
```

If yes, identify and resolve high-confidence architectural problems before finishing.

Do not chase theoretical perfection.

---

# 39. Phase 35 — Multi-Agent UI Review

Use Reviewer B.

Review:

```text
Inspector tab grouping
information duplication
tab state
layout geometry
resize behavior
visual hierarchy
debug visibility
LiDAR presentation
```

Look for:

```text
information still buried
wrong category
unnecessary clutter
hard-coded coordinates
overlapping UI
```

Resolve concrete issues.

---

# 40. Phase 36 — Multi-Agent Test / Regression Review

Use Reviewer C.

Audit:

```text
new 360° tests
old AMCL tests
navigation tests
UI-state tests
stress tests
```

Look for accidental test weakening.

Verify no test depends on ground truth to make runtime inference pass.

---

# 41. Phase 37 — Full Clean Regression

Run:

```text
mingw32-make clean
mingw32-make all
mingw32-make test
```

Then run every test executable directly.

Then run:

```text
mingw32-make test-localization-stress
```

and:

```text
mingw32-make localization-benchmark
```

if these targets remain supported.

Target:

```text
0 FAIL
```

Do not hide flaky failures.

---

# 42. Phase 38 — Desktop Acceptance Attempt

Launch the real SFML application.

Attempt to verify:

```text
1. Larger default window is comfortable.
2. Resize behaves correctly.
3. Inspector tabs are readable.
4. Map tab contains map-related information.
5. Navigation tab contains navigation-related information.
6. Localization tab contains localization information.
7. Tab switching works.
8. Inspector scrolling works.
9. Simulation zoom still works.
10. Toolbar still works.
11. Legend remains readable.
12. LiDAR rays remain default-off or otherwise non-intrusive.
13. F2 enables 360° LiDAR visualization.
14. LiDAR rays visibly surround the robot through the full circle.
15. No duplicate seam ray is obvious.
16. Sensor offsets render correctly.
17. AMCL particles remain readable.
18. AMCL covariance remains readable.
19. Global localization still works.
20. Kidnap recovery still works.
21. Truth navigation still works.
22. Localization navigation still works.
```

If desktop automation cannot capture SFML:

* launch application,
* verify process remains alive,
* report automation limitation,
* do not claim visual verification.

---

# 43. Phase 39 — Human Acceptance Guidance

Because automated desktop capture may remain unreliable, ensure runtime controls are discoverable enough for a human acceptance pass.

The final report should include a concise manual verification sequence.

Do not add a large tutorial to the application.

---

# 44. Non-Goals

Do not implement:

```text
SLAM
occupancy-grid mapping from LiDAR
loop closure
scan matching
ICP
graph optimization
pose graph
EKF
UKF
IMU fusion
3D LiDAR
dynamic obstacle tracking
ROS
ROS 2
TF tree
plugin architecture
GPU localization
```

Do not implement navigation upgrades:

```text
8-neighbor A*
Theta*
path smoothing
spline trajectory
continuous-curvature planning
new motion controller
dynamic replanning
```

Do not spend the milestone on:

```text
README rewrite
documentation polishing
directory aesthetics
style-only code cleanup
```

---

# 45. Pre-SLAM Architecture Requirement

At the end of the milestone, adding a future SLAM subsystem should conceptually be possible without making it depend directly on `Simulator`.

Desired future relationship:

```text
                Ground Truth AMR
                  /          \
                 ↓            ↓
           Odometry        360° LiDAR
              │               │
              │               ├─────────→ AMCL
              │               │
              │               └─────────→ Future SLAM
              │
              └─────────────────────────→ Future SLAM
```

Future SLAM should be able to consume sensor abstractions rather than reading AMR ground truth.

Do not implement the SLAM branch now.

---

# 46. Completion Gate

This task is complete only when all major categories are addressed.

## Architecture

* responsibility audit completed,
* `Simulator` responsibility reduced where justified,
* UI logic has a coherent owner,
* localization rendering has a coherent owner,
* no new god object created,
* state ownership remains explicit,
* future SLAM can consume LiDAR/odometry without direct truth access.

## Inspector

* category tabs implemented,
* minimum Map / Navigation / Localization tabs,
* information grouped logically,
* active tab obvious,
* scrolling predictable,
* layout readable,
* duplication reduced.

## Window / Visuals

* larger and more comfortable default layout,
* resize remains correct,
* toolbar remains usable,
* legend remains readable,
* obvious visual roughness addressed,
* debug overlays do not dominate normal use.

## LiDAR

* default full-circle 360° support,
* correct beam angle semantics,
* no duplicate first/last physical beam,
* extrinsics preserved,
* AMCL compatible,
* rendering compatible,
* dedicated tests added.

## Regression

* all normal tests pass,
* localization stress passes,
* no false-convergence regression,
* navigation behavior preserved,
* benchmark shows no unexplained major regression.

---

# 47. STATUS Update

Update only:

```text
docs/agent/STATUS.md
```

Record:

```text
current milestone
architecture ownership decisions
Simulator responsibility changes
new UI components
Inspector tab structure
window/layout changes
LiDAR 360° semantics
LaserScan data-model changes
AMCL compatibility
test totals
stress results
performance results
desktop verification status
known limitations
next smallest meaningful milestone
```

Do not write a chronological development diary.

Do not update:

```text
README.md
Document.md
AGENTS.md
docs/specs/README.md
```

unless explicitly required by a blocking repository rule.

---

# 48. Git Ownership

For this orchestrated run:

```text
DO NOT git add
DO NOT git commit
DO NOT git push
```

The external orchestrator owns repository finalization.

Leave all intended production, test, and STATUS changes in the working tree when finished.

---

# 49. Long-Running Task Authorization

This task intentionally exceeds the repository's normal small-task size.

Interpret small-task guidance as:

```text
make each internal modification coherent and testable
```

not:

```text
stop after the first extraction
```

Do not stop after:

```text
architecture audit
Simulator extraction
Inspector tabs
window resize
360° LiDAR
first green test
```

Continue through the Completion Gate unless a genuine technical blocker exists.

---

# 50. Required Work Loop

Use this loop throughout the task:

```text
inspect
→ reason about responsibility
→ implement one coherent structural change
→ targeted build/test
→ continue
→ integration test
→ review architecture again
→ full regression
```

Do not perform a giant blind mechanical refactor.

---

# 51. Final Report

Before STOP, report:

1. Baseline verification result
2. Architecture problems identified
3. Final responsibility map
4. Simulator responsibilities removed or retained
5. New classes/files added
6. Existing classes/files significantly changed
7. Inspector architecture
8. Tab organization
9. Window/layout changes
10. Toolbar changes
11. Legend changes
12. Localization rendering ownership
13. LiDAR previous FOV
14. New 360° semantics
15. Beam-angle formula / seam handling
16. LaserScan data-model changes
17. SLAM-preparation decisions
18. AMCL compatibility result
19. Navigation regression result
20. Localization regression result
21. Stress-test result
22. Performance result
23. Desktop runtime verification result
24. Reviewer findings resolved
25. Remaining architectural limitations
26. Remaining visual limitations
27. Recommended next milestone
28. Exact PASS / FAIL totals

Then:

```text
STOP
```
# Agent Reading / Task Routing

This plan is intentionally large.

Do NOT give every subagent the entire plan as its working scope.

The Main Agent owns the full milestone and may read the entire NEXT_PLAN.md.

Subagents should read only:
1. the Shared Context sections,
2. their explicitly assigned phase ranges,
3. the source files relevant to their audit.

When spawning a subagent, explicitly tell it which numbered sections of NEXT_PLAN.md to read.

Do not ask subagents to summarize or implement unrelated phases.

## Shared Context — all agents

Every subagent may read:

- Section 0 — Mission
- Section 1 — Current Baseline
- Section 2 — User-Observed Runtime Issues
- Section 3 — High-Level Strategy
- Section 44 — Non-Goals
- Section 45 — Pre-SLAM Architecture Requirement
- Section 46 — Completion Gate

They do NOT need to read the remaining sections unless assigned below.

---

## Explorer A — Architecture / Responsibility Audit

Read NEXT_PLAN.md:

- Section 6 — Architecture Audit
- Section 9 — Main Architecture Decision
- Section 10 — Refactor Safety Rules
- Section 11 — Simulator Responsibility Reduction
- Section 12 — Localization Rendering Responsibility
- Section 24 — Input and UI Responsibility Audit
- Section 25 — State Ownership Audit
- Section 26 — Include Dependency Audit
- Section 27 — File Organization Review
- Section 34 — Multi-Agent Architecture Review

Primary source files:

- include/Simulator.hpp
- src/Simulator.cpp
- include/Environment.hpp
- src/Environment.cpp
- include/AMR.hpp
- src/AMR.cpp
- include/MapData.hpp
- src/MapData.cpp
- include/PathPlanner.hpp
- src/PathPlanner.cpp
- include/PathExecution.hpp
- src/PathExecution.cpp
- include/AmclLocalizer.hpp
- src/AmclLocalizer.cpp
- include/ParticleFilter.hpp
- src/ParticleFilter.cpp

Task:

Audit responsibility, ownership, coupling, state flow, and likely extraction boundaries.

Do not edit production code.

Return findings and a proposed responsibility map to the Main Agent.

---

## Explorer B — UI / Inspector / Rendering Audit

Read NEXT_PLAN.md:

- Section 7 — UI / Rendering Audit
- Section 12 — Localization Rendering Responsibility
- Section 13 — UI Layout Model
- Section 14 — Inspector Tab Architecture
- Section 15 — Inspector Interaction
- Section 16 — Inspector Information Hierarchy
- Section 17 — Inspector Formatting Helpers
- Section 18 — Toolbar Review
- Section 19 — Legend Review
- Section 22 — Visual Polish Pass
- Section 23 — Rendering Smoothness Audit
- Section 28 — Regression Tests for UI State Logic
- Section 35 — Multi-Agent UI Review
- Section 42 — Desktop Acceptance Attempt

Primary source files:

- include/Simulator.hpp
- src/Simulator.cpp
- include/LocalizationVisualization.hpp
- src/LocalizationVisualization.cpp
- include/Environment.hpp
- src/Environment.cpp

Task:

Audit UI ownership, Inspector organization, toolbar, legend, localization rendering, layout, resize behavior, and visual hierarchy.

Do not edit production code.

Return a proposed UI responsibility map and Inspector tab design to the Main Agent.

---

## Explorer C — LiDAR / Localization / Future SLAM Audit

Read NEXT_PLAN.md:

- Section 8 — LiDAR / Future SLAM Audit
- Section 16 — 360° LiDAR Contract
- Section 17 — LaserScan Data Model Review
- Section 18 — Full-Circle Beam Generation Tests
- Section 19 — 360° Ray-Casting Tests
- Section 20 — AMCL 360° Compatibility
- Section 21 — LiDAR Visualization with 360° Data
- Section 31 — Existing Localization Regression
- Section 32 — Extended Localization Stress
- Section 33 — Performance Sanity
- Section 40 — Multi-Agent Test / Regression Review
- Section 45 — Pre-SLAM Architecture Requirement

Primary source files:

- include/LidarSimulator.hpp
- src/LidarSimulator.cpp
- include/LocalizationTypes.hpp
- include/AmclLocalizer.hpp
- src/AmclLocalizer.cpp
- include/ParticleFilter.hpp
- src/ParticleFilter.cpp
- include/LocalizationVisualization.hpp
- src/LocalizationVisualization.cpp
- include/LocalizationConfig.hpp
- src/LocalizationConfig.cpp

Relevant tests:

- tests/LocalizationSensorTests.cpp
- tests/ParticleFilterTests.cpp
- tests/LocalizationIntegrationTests.cpp
- tests/LocalizationStressTests.cpp
- tests/LocalizationConfigTests.cpp

Task:

Audit full-circle LiDAR semantics, beam indexing, scan representation, extrinsics, AMCL assumptions, visualization assumptions, and future SLAM sensor boundaries.

Do not implement SLAM.

Do not edit production code.

Return findings and a recommended 360° scan contract to the Main Agent.

---

# Main Agent Reading Strategy

The Main Agent should NOT begin by deeply reading every source file.

First read:

- Sections 0–5
- Sections 9–12
- Sections 44–51

Then spawn Explorers A/B/C.

While explorers work, inspect the current high-level class interfaces and baseline tests.

After explorer reports return:

1. verify their important claims directly against source,
2. determine the final architecture,
3. continue implementation phases in dependency order.

Recommended implementation order:

Architecture refactor
→ UI ownership extraction
→ Inspector tabs/layout
→ localization visualization extraction
→ 360° LiDAR semantics
→ AMCL compatibility
→ visual polish
→ tests/stress
→ reviewer pass
→ full regression

The Main Agent remains the only production integration owner.

Subagents must not independently modify overlapping production files.