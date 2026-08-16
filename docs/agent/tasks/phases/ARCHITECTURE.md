# Architecture Work Package

## Mission

Audit and improve responsibility boundaries before additional SLAM/navigation features are added.

This package belongs primarily to Explorer A and the Main Agent.

Explorer A performs read-only analysis.

The Main Agent verifies findings and performs production changes.

---

## 1. Required Source Reading

Prioritize:

```
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

include/AmclLocalizer.hpp
src/AmclLocalizer.cpp

include/ParticleFilter.hpp
src/ParticleFilter.cpp

include/LocalizationVisualization.hpp
src/LocalizationVisualization.cpp
```

Inspect additional files only when dependency tracing requires it.

---

## 2. Responsibility Audit

For every major class identify:

```
owned state
owned behavior
input dependencies
output dependencies
rendering responsibility
input/event responsibility
algorithmic responsibility
persistence responsibility
coordination responsibility
```

Pay special attention to:

```
Simulator
Environment
AmclLocalizer
ParticleFilter
LocalizationVisualization
```

---

## 3. Problems to Search For

Identify concrete instances of:

* god-object behavior,
* duplicated runtime state,
* persistent/runtime state mixing,
* UI logic inside domain logic,
* rendering inside algorithm code,
* algorithm behavior inside rendering code,
* state mutated by visualization,
* excessive knowledge of unrelated subsystems,
* repeated formatting/layout calculations,
* helpers living in inappropriate classes,
* ownership that would make SLAM integration difficult.

Do not report theoretical style complaints without concrete consequences.

---

## 4. Simulator Audit

Determine which current `Simulator` responsibilities genuinely belong there.

Expected responsibilities that may remain:

```
application lifecycle
high-level event routing
subsystem coordination
mode transitions
simulation timing
top-level render ordering
```

Likely candidates for extraction:

```
Inspector presentation
Inspector interaction state
generic panel layout
localization overlay drawing
toolbar layout/presentation
legend presentation
```

Do not automatically extract every helper.

A function should move only if another component becomes a clearly better owner.

---

## 5. Environment Audit

Determine whether `Environment` remains coherently responsible for:

```
map editor interaction
map-related rendering
selection/editor presentation
```

Check whether unrelated application/UI state has leaked into it.

Do not move map ownership away from `MapData`.

---

## 6. Localization Responsibility

Preserve conceptual boundaries:

```
LidarSimulator
    measurement simulation

OdometrySimulator
    measurement simulation

MapLikelihoodField
    derived map representation

ParticleFilter
    particle inference

AmclLocalizer
    localization lifecycle/orchestration

LocalizationVisualization
    presentation only
```

Verify that:

* visualization cannot affect weights,
* visualization cannot consume inference RNG,
* ground truth is not used as an estimator input,
* diagnostic truth-error values remain display/test-only.

---

## 7. State Ownership

Verify exactly one authoritative owner for major state:

```
MapData
    persistent map

AMR
    truth pose

PathExecution
    route progress

AmclLocalizer
    localization lifecycle/belief interface

UI component
    selected Inspector tab
    UI scroll/presentation state
```

UI components must not own algorithm state.

---

## 8. Header / Dependency Audit

Review relevant headers for unnecessary coupling.

Use forward declarations only where they materially improve boundaries.

Do not perform broad include cleanup unrelated to the architecture goal.

Avoid exposing implementation-only dependencies in public interfaces when a simpler boundary is practical.

---

## 9. File Organization

Do not reorganize the repository merely because it currently has:

```
include/
src/
tests/
```

Class responsibility matters more than folder cosmetics.

Do not perform a large path migration unless it clearly improves future maintainability enough to justify the churn.

If current top-level layout remains acceptable, keep it.

---

## 10. Explorer A Deliverable

Return to Main Agent:

### A. Current responsibility map

For each major class:

```
current owner
current responsibilities
problematic responsibilities
```

### B. High-confidence problems

Rank:

```
Critical
Important
Optional
```

### C. Proposed responsibility map

Identify:

* what stays,
* what moves,
* suggested coherent components,
* dependencies between them.

### D. Implementation order

Recommend structural changes in dependency order.

### E. Risks

Identify behavior most likely to regress.

Do not modify production code.

---

## 11. Main Agent Architecture Decision

After Explorer A returns:

The Main Agent must verify important findings against source.

Do not blindly follow recommendations.

Create a final internal architecture plan.

The goal is:

```
clearer ownership
lower coupling
future SLAM extension capability
```

not:

```
maximum number of classes
minimum lines per file
```

---

## 12. Structural Refactor Loop

For each coherent extraction:

```
inspect exact responsibility
        ↓
move state/behavior
        ↓
compile
        ↓
run targeted tests
        ↓
continue
```

Avoid giant mechanical moves.

---

## 13. UI Boundary Preparation

Before UI implementation begins, architecture should make it possible for presentation components to own:

```
Inspector state
Inspector rendering
Inspector interaction
localization presentation
```

without owning:

```
MapData
AMCL belief
PathExecution progress
robot truth
```

They receive/read state and present it.

Do not build a generic GUI framework.

---

## 14. Future SLAM Check

At the end of architectural refactor ask:

> Could a future SLAM subsystem consume odometry + LaserScan without being wired directly into AMR truth or Inspector code?

If no, investigate why.

Resolve high-confidence blockers now.

Do not implement SLAM.

---

## 15. Architecture Review

After all implementation, a fresh reviewer should inspect:

```
Simulator responsibility
Environment responsibility
UI ownership
rendering ownership
localization ownership
sensor ownership
state ownership
future SLAM extension points
```

Ask explicitly:

> Would adding a SLAM subsystem now force major unrelated changes?

Resolve concrete high-confidence findings.

Do not pursue theoretical architectural perfection.

