# UI / Inspector Refactor Work Package

## Mission

Reorganize the current desktop UI so that growing navigation and localization diagnostics remain usable.

Primary user-observed problem:

The right Inspector is too long and mixes unrelated information.

This package owns:

```
Inspector
layout
window sizing
toolbar presentation
legend presentation
localization visualization presentation
visual polish
```

It does not own localization inference or navigation algorithms.

---

## 1. Explorer B Reading Scope

Inspect primarily:

```
include/Simulator.hpp
src/Simulator.cpp

include/Environment.hpp
src/Environment.cpp

include/LocalizationVisualization.hpp
src/LocalizationVisualization.cpp
```

Inspect related UI/test helpers only when necessary.

Do not audit KLD, recovery mathematics, path planning, or unrelated localization internals.

---

## 2. Initial UI Audit

Determine current ownership of:

```
window geometry
simulation viewport
Inspector geometry
Inspector text generation
Inspector interaction
Inspector scrolling
toolbar
legend
localization overlays
LiDAR overlays
selection visuals
resize handling
```

Identify which responsibilities still live directly in `Simulator`.

Explorer B is read-only during this audit.

---

## 3. Inspector Target

Replace the single long vertical information stream with tabs.

Minimum:

```
Map
Navigation
Localization
```

A fourth tab is allowed only if information architecture clearly requires it.

Do not create tabs for symmetry alone.

---

## 4. Map Tab

Recommended information:

```
Cursor
Map Stats
Map Validation
Selected Object
Start
Goal
map-related editor state
```

Avoid unrelated navigation/localization metrics.

---

## 5. Navigation Tab

Recommended information:

```
Robot State
Navigation Mode
Planning Start Source
Path Planning
Path Execution
Current waypoint/progress
Stop Reason
Start / Goal where relevant
```

Do not duplicate extensive localization diagnostics here.

If AMCL navigation is active, concise indication of localization gating is appropriate.

---

## 6. Localization Tab

Organize information hierarchically.

### Primary state

```
State
Support
Initialization
```

### Estimate

```
X
Y
Yaw
```

### Confidence

```
Position sigma
Heading sigma
Dominant cluster weight
Second cluster weight
```

### Particle Filter

```
Particle count
ESS
Entropy
Cluster count
Significant clusters
```

### Sensor

```
Selected beams
Used beams
Skipped beams
Invalid beams
Max-range beams
Observation quality
Likelihood contrast
```

### Recovery

```
Recovery probability
slow weight average
fast weight average
```

### Diagnostics

```
Odometry pose
Ground-truth pose
Position error
Heading error
```

Truth diagnostics remain presentation/test-only.

### Layers / Controls

Show concise discoverability hints for relevant localization visualization hotkeys.

Avoid turning the toolbar into a debug-button wall.

---

## 7. Inspector Interaction

Required:

```
click tab to activate
active tab visually clear
only active body displayed
selection persists during runtime
scroll behavior deterministic
```

Preferred:

Each tab maintains independent scroll offset.

Acceptable fallback:

Reset scroll on tab switch.

Either behavior must be deterministic and testable.

---

## 8. Mouse Input Isolation

Mouse wheel over Inspector:

```
scroll Inspector
```

Mouse wheel over simulation viewport:

```
retain map zoom
```

Do not allow event leakage between regions.

Clicks on tabs must not trigger map/editor actions.

---

## 9. Window / Layout

Increase default window size to suit current information density.

Review:

```
default width
default height
minimum practical dimensions
Inspector width
viewport ratio
toolbar region
legend region
```

Do not design only for one resolution.

Resize must recompute layout correctly.

---

## 10. Inspector Width

The current information density justifies a wider panel.

Choose a width based on actual content readability.

Avoid wasting excessive simulation viewport space.

The design should remain usable on ordinary desktop displays.

---

## 11. Presentation Component

Architecture may introduce a coherent UI component such as:

```
InspectorPanel
```

or equivalent.

A useful component may own:

```
active tab
scroll state
layout geometry
text/section presentation
Inspector input handling
```

It must not own:

```
MapData
AMCL state
navigation execution state
robot truth
```

It receives the current state required for rendering.

Do not build a generic GUI framework.

---

## 12. Structured Inspector Content

Reduce large repeated ad-hoc formatting blocks.

A small presentation model is allowed, for example:

```
InspectorSection
    title
    lines
    optional status style
```

Use only if it improves readability.

Do not over-engineer.

---

## 13. Localization Visualization

Review whether `LocalizationVisualization` should become the coherent owner of:

```
particle rendering
AMCL estimate marker
covariance ellipse
odometry marker
LiDAR hit points
LiDAR rays
localization visual style
```

It must remain observational.

It must never:

```
update particles
change weights
change AMCL state
consume inference RNG
```

---

## 14. Toolbar

Preserve all editor functionality.

Improve:

```
spacing
grouping
active state
readability
resize behavior
```

Do not add a button for every debug hotkey.

Prefer concise hints in the relevant Inspector tab.

---

## 15. Legend

Keep useful identity for:

```
Robot
Start
Goal
Path
Particles
AMCL
Odometry
LiDAR
```

Review placement.

It must not cover important map content unnecessarily.

Compact presentation is preferred.

---

## 16. Visual Polish

Review:

```
grid contrast
obstacle contrast
Start / Goal markers
path visibility
particle alpha
AMCL estimate marker
covariance
odometry
LiDAR hit points
selection outline
tab highlighting
section headers
spacing
padding
```

Fix concrete roughness.

Do not spend large effort on animation.

---

## 17. Rendering Smoothness Audit

Investigate obvious visual roughness potentially caused by:

```
unnecessary geometry rebuild
integer rounding
view transforms
frame-dependent presentation
abrupt marker state
poor path marker placement
particle rendering
```

Fix presentation defects.

Do not redesign the motion controller.

Stop-turn-go robot motion remains a future navigation problem if still visible.

---

## 18. UI Tests

Test non-pixel logic where practical:

```
default active tab
tab switching
tab persistence
scroll state
layer toggle state
layout calculations
resize calculations
navigation mode state
global/local localization controls
```

Do not build screenshot-based unit infrastructure.

---

## 19. Explorer B Deliverable

Return:

1. current UI responsibility problems,
2. proposed UI component boundaries,
3. Inspector tab design,
4. information grouping,
5. resize/layout problems,
6. localization rendering ownership recommendation,
7. implementation risks.

Do not modify production code during the audit.

---

## 20. UI Reviewer

After implementation review:

```
Inspector grouping
duplication
tab state
scrolling
layout geometry
resize behavior
hard-coded coordinates
visual hierarchy
debug visibility
LiDAR presentation
```

Resolve concrete regressions or obvious usability defects.

---

## 21. Desktop Acceptance

Attempt to verify:

```
larger default window
resize
Map tab
Navigation tab
Localization tab
tab switching
Inspector scrolling
map zoom
toolbar
legend
particle display
AMCL estimate
covariance
LiDAR toggle
navigation modes
```

If SFML automation cannot capture the window:

* launch it,
* verify process stays alive,
* report automation limitation,
* do not claim visual success.

