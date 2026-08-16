# Milestone: Pre-SLAM Architecture, UI, and 360° LiDAR Refactor

## 0. Mission

Perform a deliberate architecture and presentation refactor before beginning SLAM work.

The repository already contains working:

* map editing and persistence,
* A* path planning,
* clearance-aware navigation,
* path execution,
* AMR runtime simulation,
* simulated odometry,
* simulated LiDAR,
* AMCL particle filtering,
* likelihood-field localization,
* KLD adaptive sampling,
* recovery,
* clustering and ambiguity handling,
* global localization,
* kidnapped-robot recovery,
* optional localization-driven navigation,
* localization visualization and diagnostics.

This milestone must **not** add another major algorithm.

Primary goals:

1. Audit subsystem responsibilities.
2. Reduce `Simulator` responsibility and coupling.
3. Reorganize the right Inspector into category tabs.
4. Improve desktop layout and visual hierarchy.
5. Convert LiDAR to clean 360° full-circle semantics.
6. Prepare sensor abstractions for future SLAM.
7. Preserve existing navigation and AMCL behavior.
8. Preserve or improve testability.
9. Perform architecture, UI, and regression review after implementation.

This is intentionally a large orchestrated task.

Do not stop after the first successful refactor or first green test.

---

## 1. Baseline

Expected normal regression:

```
164 PASS
0 FAIL
```
Expected extended localization stress:

```
4 PASS
0 FAIL
```

Stable simulation-truth navigation remains the default.

Localization-driven navigation remains optional.

Ground truth must remain unavailable to localization inference.

---

## 2. User-Observed Problems

### 2.1 Inspector overload

The right Inspector contains too many unrelated values in one long stream.

Target conceptual structure:

```
Map | Navigation | Localization
```

Information must be categorized and visually hierarchical.

### 2.2 Window/layout density

Review:

* default window size,
* Inspector width,
* simulation viewport,
* resize behavior,
* scrolling,
* toolbar spacing,
* legend placement.

The application should feel significantly less cramped.

### 2.3 Visual roughness

Address clear presentation defects involving:

* spacing,
* alignment,
* transparency,
* marker scale,
* localization overlays,
* tab state,
* debug overlays,
* text hierarchy.

Do not redesign the product.

### 2.4 LiDAR FOV

Current LiDAR is not full-circle.

The future SLAM path requires clean 360° planar scan semantics.

SLAM itself remains out of scope.

---

## 3. Task Package Structure

This master plan intentionally contains only milestone-level coordination.

Detailed work is located under:

```
tasks/
├─ NEXT_PLAN.md
└─ phases/
   ├─ ARCHITECTURE.md
   ├─ UI_REFACTOR.md
   ├─ LIDAR_360.md
   └─ VERIFICATION.md
```

The Main Agent owns the whole milestone.

Subagents must **not** read every work package.

---

## 4. Agent Routing

### Main Agent

The Main Agent is:

* architecture owner,
* production integration owner,
* final decision maker,
* test/regression owner.

Read first:

```
NEXT_PLAN.md
AGENTS.md
docs/agent/STATUS.md
```

Also inspect relevant specifications required by repository rules.

Do **not** begin by deeply reading every implementation file.

First establish baseline, then spawn audit agents.

---

### Explorer A — Architecture

Read only:

```
NEXT_PLAN.md
phases/ARCHITECTURE.md
```

Focus on responsibility, ownership, coupling, state flow, and future SLAM architecture.

Do not modify production code.

---

### Explorer B — UI / Inspector

Read only:

```
NEXT_PLAN.md
phases/UI_REFACTOR.md
```

Focus on Inspector, rendering responsibility, layout, toolbar, legend, resize behavior, and visual hierarchy.

Do not modify production code.

---

### Explorer C — LiDAR / Localization

Read only:

```
NEXT_PLAN.md
phases/LIDAR_360.md
```

Focus on 360° scan semantics, LaserScan abstraction, AMCL assumptions, visualization, sensor tests, and future SLAM sensor boundary.

Do not modify production code during the initial audit.

---

### Final Reviewers

After implementation:

* Architecture reviewer reads `ARCHITECTURE.md`.
* UI reviewer reads `UI_REFACTOR.md`.
* Sensor/test reviewer reads `LIDAR_360.md` and relevant portions of `VERIFICATION.md`.

Do not give reviewers the complete task package unless necessary.

---

## 5. Multi-Agent Rules

Use at most three useful concurrent subagents.

If spawning fails because of agent limits:

* continue serially,
* do not repeatedly retry,
* do not stop the milestone.

Parallel agents are primarily for:

```
audit
analysis
review
```

The Main Agent remains the production integration owner.

Do not allow multiple agents to independently rewrite overlapping files such as:

```
src/Simulator.cpp
include/Simulator.hpp
include/LocalizationTypes.hpp
```

---

## 6. Required Execution Order

Use this dependency order:

```
Baseline verification
        ↓
Explorer A/B/C audits
        ↓
Main architecture decision
        ↓
Architecture refactor
        ↓
UI ownership extraction
        ↓
Inspector tabs / layout
        ↓
Localization visualization cleanup
        ↓
360° LiDAR
        ↓
AMCL compatibility
        ↓
Visual polish
        ↓
Targeted regression
        ↓
Extended localization stress
        ↓
Architecture/UI/test reviewers
        ↓
Full clean regression
        ↓
Desktop acceptance attempt
        ↓
STATUS
```

Do not implement UI first if the architecture audit shows that the current UI ownership must be changed.

---

## 7. Refactor Safety Rules

During structural changes:

```
NO SLAM
NO 8-neighbor A*
NO Theta*
NO path smoothing
NO controller redesign
NO new localization algorithm
NO scan matching
NO occupancy-grid mapping
```

Preserve existing behavior unless this task explicitly changes it.

After every major extraction:

```
build
targeted tests
```

Do not wait until the end to discover structural breakage.

---

## 8. Desired Responsibility Direction

The final architecture should roughly preserve:

```
MapData
    persistent map/domain state

Environment
    editor-facing map interaction/environment rendering

AMR
    ground-truth robot pose and body geometry

PathPlanner
    planning algorithm

PathExecution
    execution progress

LidarSimulator
    LiDAR measurement simulation

OdometrySimulator
    odometry measurement simulation

MapLikelihoodField
    derived localization map representation

ParticleFilter
    particle belief algorithm

AmclLocalizer
    localization lifecycle

UI / visualization components
    presentation and interaction only

Simulator
    application orchestration and subsystem coordination
```

`Simulator` must not remain the owner of large amounts of:

```
Inspector formatting
tab rendering
localization drawing math
generic UI layout calculations
algorithmic localization calculations
```

Do not create abstractions merely to reduce line count.

---

## 9. Future SLAM Boundary

The architecture after this milestone should support the conceptual future:

```
                Ground Truth AMR
                  /          \
                 ↓            ↓
           Odometry        360° LiDAR
              │               │
              │               ├──────→ AMCL
              │               │
              │               └──────→ Future SLAM
              │
              └──────────────────────→ Future SLAM
```

Future SLAM must consume sensor abstractions.

It must not require direct access to AMR ground truth.

Do not implement the future SLAM subsystem now.

---

## 10. Completion Gate

This milestone is complete only when:

### Architecture

* responsibility audit completed,
* concrete ownership problems resolved,
* `Simulator` reduced where justified,
* no replacement god object created,
* state ownership remains explicit,
* rendering does not mutate inference/domain state.

### UI

* larger usable default layout,
* Inspector has category tabs,
* at least Map / Navigation / Localization tabs,
* information is grouped coherently,
* Inspector scrolling works,
* map viewport behavior remains correct,
* resize is usable,
* toolbar remains functional,
* visual hierarchy is improved.

### LiDAR

* default scan is full 360°,
* no duplicated first/last physical ray,
* sensor extrinsics still work,
* AMCL accepts the new scan,
* visualization works with full-circle data,
* dedicated tests cover beam geometry.

### Regression

* normal tests pass,
* localization stress passes,
* navigation behavior preserved,
* false-convergence protection preserved,
* benchmark has no unexplained major regression.

Target:

```
0 FAIL
```

---

## 11. Documentation

Update only:

```
docs/agent/STATUS.md
```

Record:

* architecture ownership decisions,
* extracted responsibilities,
* Inspector architecture,
* layout changes,
* LiDAR semantics,
* LaserScan changes,
* AMCL compatibility,
* tests,
* stress results,
* benchmark,
* desktop verification,
* known limitations,
* next meaningful milestone.

Do not write a chronological development diary.

---

## 12. Git Ownership

Codex must not run:

```
git add
git commit
git push
```

The external orchestrator owns repository finalization.

---

## 13. Long-Running Authorization

Repository small-task guidance does not mean stopping after one extraction.

For this explicitly orchestrated milestone:

```
small/correct/testable internal changes
```

are required, but continue until the milestone Completion Gate is satisfied.

---

## 14. Final Report

Before STOP report:

1. baseline result,
2. architecture findings,
3. final responsibility map,
4. files/classes added,
5. major responsibilities moved,
6. Inspector architecture,
7. window/layout changes,
8. localization rendering changes,
9. LiDAR old/new semantics,
10. LaserScan changes,
11. future-SLAM boundary,
12. navigation regression,
13. AMCL regression,
14. stress results,
15. benchmark,
16. desktop verification,
17. reviewer findings,
18. unresolved limitations,
19. exact PASS / FAIL totals.

Then:

```
STOP
```
