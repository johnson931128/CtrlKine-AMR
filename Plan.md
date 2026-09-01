# Next UI/UX Milestone Plan

This document is analysis and implementation planning only. It does not approve
or start an implementation phase.

## 1. Current Architecture Findings

### Repository and verification baseline

- The repository was fetched before this analysis. Local `main` and
  `origin/main` are synchronized at `8fb248f` (`0` ahead, `0` behind).
- The worktree was clean before this file was added.
- `docs/agent/STATUS.md` records a clean application build, 226/226 normal
  regression, 4/4 localization stress, and 5/5 SLAM stress after the source
  layout refactor. Those results were inspected, not rerun in this analysis-only
  turn.
- `docs/specs/README.md` still marks `EditorUISpec.md` as not started. The next
  milestone therefore needs an approved UI contract before production changes.
- The build path is `mingw32-make all`; the application launches as
  `build/CtrlKine-AMR.exe`. MinGW, SFML 3.0.0, and required SFML DLLs are
  available in the current environment.

### Runtime ownership

- `Simulator` owns the application loop, event routing, simulation timing,
  simulation and UI views, editor coordination, AMR truth, sensor acquisition,
  AMCL and SLAM orchestration, path execution, UI state, and top-level render
  order.
- `Environment` owns `MapData`, editor mode/tool state, map editing, map
  persistence delegation, selection hit testing, and simulation-map rendering.
- `MapData` is the authoritative editable/simulation map. `AMR` is the
  authoritative ground-truth robot pose.
- `AmclLocalizer` and `SlamFrontend` own independent inference state. The same
  immutable `SimulatorSensorFrame` is fanned out to both, while odometry, LiDAR,
  and AMCL use separate random streams.
- `LocalizationVisualization` and `SlamVisualization` are already observational
  leaf renderers. Their mutable state is limited to presentation options and
  cached SFML geometry.
- `InspectorPanel` owns the active tab and four independent scroll offsets. It
  receives read-only data and does not own domain or inference state.

### Current rendering and frame usage

The current world render order in `Simulator::render()` is:

1. `Environment` grid/map/editor content.
2. SLAM occupancy, corrected pose, and predicted pose.
3. LiDAR rays/hits.
4. AMCL particles, estimate, covariance, and odometry.
5. Active path.
6. Ground-truth AMR and selection outline.
7. Toolbar and Inspector in the UI view.

Important frame details:

- `LocalizationVisualization::drawScan()` receives
  `m_scanGroundTruthPose`. The current LiDAR overlay is therefore positioned
  from truth for display.
- AMCL particles and estimates, odometry, and path geometry are represented in
  the simulator/map world frame.
- SLAM inference stays in its local frame. `SlamVisualization` applies
  `m_slamDisplayOrigin`, captured from truth at SLAM bootstrap/reset, only to
  align the display. This is allowed by `SlamV1Spec.md` only as a clearly
  display-only transform.
- The current single composite canvas hides these frame differences. Moving the
  layers into a Robotics view without an explicit frame policy would preserve a
  subtle truth dependency in presentation and make Camera Follow ambiguous.

### Current layout, toolbar, Inspector, and camera

- `ApplicationLayout` calculates only three rectangles: toolbar, simulation
  viewport, and Inspector. The default is 1440x900 with a 360-pixel Inspector.
- `EditorToolbar` owns seven editor button rectangles, hit testing, active style,
  and text drawing.
- Toolbar labels use the first loadable Windows font from Segoe UI, Arial,
  Calibri, and Consolas; the selected font is not exposed diagnostically.
- Toolbar text uses character size 14. Horizontal centering uses only
  `getLocalBounds().size.x`, ignores the bounds origin, can produce fractional
  coordinates, and vertical placement is a fixed `+8` rather than true glyph
  bounds centering.
- SFML font texture smoothing is enabled by default. The UI view is otherwise a
  1:1 pixel-space view, so unintended world-view scaling is not the primary
  cause at the default size.
- `InspectorPanel` is already sectioned by four tabs, but every section is a
  title plus one newline-delimited string. Labels and values therefore cannot
  form stable columns, and long fields such as Reason, Message, and stop reason
  compete with ordinary scalar rows.
- Camera state is a single `sf::View` (`m_simView`). Resize preserves center and
  derives a zoom factor from the previous default size. Wheel zoom and drag pan
  are implemented directly in `Simulator::processEvents()`. There is no follow
  state or separate per-view camera.

### Existing tests

- `SimulatorRuntimeTests.cpp` covers runtime seams, sensor fan-out, RNG
  independence, localization-driven navigation, layout geometry, toolbar hit
  geometry, Inspector tabs, and scroll behavior.
- `LocalizationSensorTests.cpp` covers visualization defaults, LiDAR render
  subsampling, hit semantics, and covariance geometry.
- `SlamIntegrationTests.cpp` proves the SLAM display transform is observational.
- SLAM/localization algorithm, integration, stress, and benchmark suites provide
  regression coverage but do not inspect a desktop window.
- No current automated test checks font raster quality, text centering on a
  real display, view-switch rendering, Camera Follow state, screenshot output,
  or end-to-end desktop interaction.

## 2. Problems Identified

1. The single canvas mixes simulation truth, editor content, navigation output,
   AMCL, odometry, LiDAR, and SLAM. This makes layer meaning and frame provenance
   difficult to understand.
2. `Simulator` directly owns both visualization leaf objects and every render
   decision. Adding further robotics layers will keep expanding its presentation
   responsibility.
3. A simple `if (Robotics)` around existing draw calls is unsafe: current LiDAR
   placement reads truth, while SLAM uses a truth-derived display transform.
4. A Robotics camera cannot be specified correctly until the estimated follow
   source and display frame are explicit.
5. A single camera would cause Simulation pan/zoom/follow state to leak into the
   Robotics view and vice versa.
6. Toolbar text is small and not centered from complete glyph bounds. Fractional
   placement, font smoothing, fallback font differences, and Windows DPI can all
   contribute to the reported blur.
7. Inspector content is structurally encoded in strings. Alignment, wrapping,
   styling, measurement, and testability are coupled.
8. The current UI regression tests validate geometry/state only. They cannot be
   used as desktop visual or interaction acceptance.
9. The installed Windows automation can launch and uniquely target the SFML
   window, and it can inject keys/clicks/scroll/drag in principle. In this
   environment its screenshot capture failed twice with
   `SetIsBorderRequired failed: unsupported interface (0x80004002)`. The SFML
   accessibility tree exposes only the native window chrome, not canvas
   controls. Safe coordinate playtesting therefore cannot currently close the
   observe-act-verify loop.

## 3. Proposed UI Architecture

The requested separation is sound, with one qualification: the milestone should
not create a symmetric `SimulationVisualizer` merely for naming symmetry. The
existing `Environment`, AMR drawing, selection drawing, and path drawing already
form a coherent Simulation render path. Extracting all of them would add churn
without improving the inference boundary.

Recommended top-level structure:

```text
Simulator
  owns lifecycle, runtime state, event routing, and view-mode coordination
  |
  +-- ApplicationLayout
  |     owns rectangles for top bar, editor tools, view controls,
  |     viewport, and Inspector
  |
  +-- ViewControlBar
  |     owns Simulation / Robotics / Follow presentation and hit testing
  |
  +-- ViewportCamera x 2
  |     owns per-view center, zoom, pan, and follow state
  |
  +-- Simulation render path
  |     Environment + path result + AMR truth + selection
  |
  +-- RoboticsVisualizer
  |     consumes a read-only presentation snapshot
  |     +-- LocalizationVisualization
  |     +-- SlamVisualization
  |     +-- LiDAR / odometry / path presentation
  |
  +-- InspectorPanel
        consumes read-only structured diagnostics
```

`Simulator` should retain top-level render ordering and mode transitions, but it
should no longer know the draw order of individual robotics layers.

## 4. RoboticsVisualizer Design

### Placement

Create a cross-subsystem presentation directory:

```text
include/visualization/
src/visualization/
```

`RoboticsVisualizer` does not belong under `slam/`, `localization/`, or `ui/`:
it composes several robotics result types, draws inside a world viewport, and is
not a widget or an inference subsystem.

### Read-only input contract

Introduce a narrow `RoboticsVisualizationData` value/reference aggregate. It may
contain:

- AMCL particles, estimate, statistics, and layer options;
- latest immutable `LaserScan`;
- an optional estimated pose at which to display the scan;
- odometry pose and initialized flag;
- `SlamUpdateResult`, `SlamOccupancyGrid`, and an explicitly named
  `slamLocalToDisplay` transform;
- presentation-ready path vertices/waypoints;
- the selected Robotics follow source/status.

It must not contain:

- mutable references or pointers;
- `AMR`;
- `Environment`;
- `Simulator`;
- `LidarSimulator` or `OdometrySimulator`;
- `AmclLocalizer` or `SlamFrontend` objects;
- RNGs, update callbacks, reset callbacks, planners, controllers, or map-edit
  commands.

The public header should depend on result/data types, not inference owners.
`SlamVisualization.hpp` should also stop including `SlamFrontend.hpp` when its
actual needs are only SLAM result/grid types.

### Internal responsibilities

`RoboticsVisualizer` should own:

- `LocalizationVisualization` and `SlamVisualization` instances;
- presentation options and layer order;
- cache invalidation/rebuild calls for particle and SLAM geometry;
- estimated scan-anchor selection according to an approved display-frame rule;
- drawing of presentation-ready navigation path geometry;
- an optional estimated follow target result.

It must not own sensor acquisition, estimator cadence, map matching, particle
updates, path planning, navigation control, or simulation state.

### First-version frame policy

Recommended policy, pending approval:

- Use the map/world display frame as the Robotics canvas frame.
- Draw AMCL, odometry, and path in their existing world coordinates.
- Draw the SLAM-local grid/poses through the existing display-only bootstrap
  transform, with that provenance labeled in Inspector.
- Draw LiDAR from an estimated anchor selected for the Robotics view, never from
  `m_scanGroundTruthPose`.
- Do not draw the ground-truth AMR or editor cursor/selection in Robotics view.
- Do not silently use truth as a fallback when an estimate is unavailable. Show
  an explicit unavailable state and retain the previous camera center.

This preserves the existing approved SLAM inference boundary while making
truth-derived display alignment visible and one-way.

## 5. Simulation / Robotics View Ownership

### Simulation view

Owns presentation of:

- editable `Environment` grid, boundary, obstacles, work zones, start, and goal;
- editor cursor preview and selection;
- ground-truth AMR body;
- the current planned/executed path as simulation/navigation output;
- Simulation camera follow targeting the ground-truth AMR pose.

It must not draw SLAM occupancy, SLAM pose, AMCL particles/covariance, odometry,
or robotics LiDAR overlays after the split.

### Robotics view

Owns presentation of:

- SLAM occupancy and corrected/predicted poses;
- AMCL particles, estimate, and covariance;
- LiDAR rays/hits anchored to an approved estimated pose;
- odometry;
- navigation path result;
- Robotics camera follow targeting an approved estimated pose.

It must not draw or query ground-truth AMR pose for camera follow. Any
truth-derived transform or error diagnostic remains explicitly display-only and
must not be returned to AMCL, SLAM, navigation, or control.

### Mode behavior

- Switching views changes rendering, active camera, pointer routing, and visible
  editor controls. It does not reset or pause simulation, AMCL, SLAM, path
  execution, or RNG state.
- Map-edit clicks and editor tool shortcuts operate only in Simulation view.
- Runtime/navigation/localization commands may remain global if intentionally
  approved, so the robot can be driven while observing Robotics view.
- Each view retains independent center, zoom, pan, and follow state.
- Inspector remains a shared diagnostics surface and does not become a large
  canvas.

## 6. Inspector Table Design

Replace the internal newline-string model with a small presentation model, not a
generic GUI framework:

```text
InspectorSection
  title
  rows: InspectorRow[]
      label
      value
      optional semantic color
  blocks: InspectorBlock[]
      label/title
      multiline text
      optional semantic color
```

Layout rules:

- Keep the existing Map, Navigation, Localization, and SLAM tabs and independent
  scrolling.
- Use one computed label column and one value column per panel width. Values are
  left aligned from a common x-coordinate; numbers are not forced into a
  monospaced font.
- Clamp the label column to a readable range rather than hard-coding positions
  for 360 pixels only.
- Measure every rendered row/block after wrapping, and derive content height from
  those measurements before scroll clamping.
- Use full-width multiline blocks for validation messages, Application Status,
  planning Message, execution stop reason, localization Reason/explanation, and
  SLAM frame/provenance notes.
- Preserve every currently displayed field and shortcut hint. Information may be
  regrouped, but not silently removed.
- Keep the legend compact in the footer or move it to a clearly bounded
  Robotics-layer section only if desktop review proves the footer is too dense.
- Inspector must receive data and format it; it must not draw maps, particle
  clouds, plots, or other large visualization.

Recommended per-tab grouping:

- Map: Cursor, Map Stats, Validation, Selected Object, Start, Goal, controls,
  application status.
- Navigation: Robot State, mode/start source, planning, execution, controls.
- Localization: Primary State, Estimate, Confidence, Particle Filter, Sensor,
  Recovery, display-only Diagnostics, layers, controls.
- SLAM: State, local pose, Scan Matching, map statistics, lifecycle, display
  provenance, layers, controls.

## 7. Toolbar Typography Fix Strategy

Do not choose a font-smoothing setting from code inspection alone. Use a small
A/B matrix and desktop screenshots.

Implementation strategy:

1. Record the loaded font path/name in a diagnostic log or debug-only label so
   fallback behavior is observable.
2. Center text using the complete local bounds:
   subtract `bounds.position`, include `bounds.size`, and round the final x/y to
   integer UI pixels.
3. Replace fixed `+8` vertical placement with true visual-bounds centering.
4. Compare character sizes 14, 15, and 16 using `Select [S]`, `Obstacle [O]`,
   `Erase [E]`, and the widest labels at default and narrow window sizes.
5. Keep the UI view in integer pixel space and pixel-snap button rectangles when
   proportional widths produce fractional coordinates.
6. Compare SFML smoothing on/off. Keep smoothing on unless screenshots show a
   repeatable improvement from disabling it; unsmoothed small glyphs may become
   jagged rather than clearer.
7. Verify Windows display scaling at the available 100%, 125%, and 150% settings
   where practical. Do not change system DPI settings automatically.
8. Apply the same centering helper to the new view controls and Inspector tabs so
   typography does not diverge.

Automated tests may prove integer alignment, bounds containment, non-overlap, and
stable geometry. Only desktop screenshots/human inspection can accept clarity.

## 8. Camera Follow Design

Introduce a small `ViewportCamera` state/controller independent of robot and
estimator classes.

Owned state:

- `sf::View`;
- default viewport size and current zoom factor;
- follow enabled flag;
- pan-in-progress flag and last pan pixel;
- last valid follow target, if any.

Operations:

- `resize(viewportRect)`: preserve center, zoom factor, and follow state;
- `zoom(factor)`: change view size only; do not disable follow;
- `beginPan(pixel)`: begin pan and immediately disable follow;
- `panTo(pixel, mapPixelToCoords callback/input)`: deterministic delta move;
- `endPan()`;
- `setFollowEnabled(bool)`;
- `recenter(target)`: set center exactly to target, without changing view size;
- `updateFollow(optionalTarget)`: if enabled and a target exists, set center
  exactly once per frame; no interpolation or damping.

Target rules:

- Simulation camera receives `AMR` ground-truth pose only while Simulation view
  is active.
- Robotics camera receives only an estimated display pose selected by
  `RoboticsVisualizer`/presentation policy.
- If the Robotics estimate is invalid, do not fall back to AMR truth. Keep the
  prior center and show Follow as waiting/unavailable.
- Re-enabling Follow recenters immediately on the current valid target and keeps
  the exact current zoom.
- Manual pan disables Follow for only the active view. View switching does not
  copy or reset camera state.
- `Ctrl+0` behavior must be decided explicitly: recommended behavior is reset
  only the active camera to its default size/center and disable Follow, rather
  than reset both hidden and visible cameras.

## 9. UI Playtest Strategy

### Required workflow after implementation

1. Run a clean application build.
2. Launch the real `build/CtrlKine-AMR.exe` from the repository working
   directory so relative config/map paths are deterministic.
3. Target the exact SFML window title/process and set a known window size.
4. Capture a baseline screenshot before input.
5. Exercise view switching, editor tools, navigation, localization/SLAM layers,
   Inspector tabs/scroll, zoom, pan, and Follow.
6. Capture screenshots after each meaningful state change.
7. Inspect clipping, overlap, label/value alignment, typography, active states,
   frame labels, camera center, and layer provenance.
8. Record issues, modify implementation, rebuild, relaunch from a clean process,
   and replay the same scenario.
9. Keep a human acceptance pass for subjective clarity and interaction feel.

### What this environment can currently do

- Build and launch prerequisites are present.
- The installed Computer Use capability can launch the exact executable,
  uniquely identify the SFML window, inject keyboard/mouse/scroll/drag input,
  and close the process.
- SFML exposes no semantic accessibility nodes for its canvas; only the native
  window chrome is discoverable. Interaction must therefore use screenshot-
  derived window-relative coordinates and refresh after each action.
- Current screenshot capture is blocked by the repeated
  `SetIsBorderRequired ... 0x80004002` error. Because there is no safe current
  observation, this turn did not attempt coordinate interaction and does not
  claim visual acceptance.
- No alternate `ffmpeg`, ImageMagick, OBS, MSS, Pillow, pyautogui, pywinauto, or
  OpenCV capture path is currently installed.

### Infrastructure options

Preferred order:

1. Repair/upgrade the Windows Graphics Capture helper or host support so the
   existing Computer Use observe-act-screenshot loop works.
2. If that cannot be made reliable, add a narrow repository-owned capture seam
   that saves the final SFML framebuffer on an explicit test command/hotkey to a
   caller-provided temporary path. It must be inactive by default and must not
   alter inference or normal runtime behavior.
3. Add `scripts/ui_playtest.ps1` only after the capture decision. It should build,
   start one process with a known working directory, wait for the exact window,
   record process/window readiness, and clean up the process. It should not claim
   visual success by itself.
4. Store transient screenshots and logs outside the tracked repository or under
   an explicitly ignored artifact directory.

A launch-only script plus process survival is a smoke test, not UI acceptance.

## 10. Automated Regression Strategy

Keep existing suites and add deterministic non-pixel coverage:

- View mode defaults, switching, and persistence.
- View-control hit testing and event isolation.
- Independent Simulation/Robotics camera center, zoom, pan, and follow state.
- Manual pan disables active-view Follow.
- Re-enable Follow recenters without changing zoom.
- Resize preserves zoom and follow state.
- Missing Robotics estimate never selects AMR truth.
- Switching views does not reset/mutate AMCL, SLAM, path execution, sensor
  frames, or RNG sequences.
- Robotics presentation input is read-only and visualization/cache rebuilds do
  not mutate inference results or maps.
- Simulation render policy excludes robotics overlays; Robotics render policy
  excludes ground-truth AMR/editor overlays.
- Inspector table measurement, wrapping, per-tab scroll, compact-height footer,
  and long Reason/Message blocks.
- Toolbar/view-control text placement is pixel-aligned and remains inside its
  button bounds for supported widths.
- Existing SLAM display-transform, LiDAR subsampling, sensor fan-out, and RNG
  isolation tests remain passing.

Prefer extending `SimulatorRuntimeTests.cpp`, `LocalizationSensorTests.cpp`, and
`SlamIntegrationTests.cpp` rather than creating screenshot assertions or a new
test framework. A separate UI-state executable is justified only if runtime-test
link dependencies become unmanageable.

Final automated sequence remains:

```text
mingw32-make clean
mingw32-make all
mingw32-make test
mingw32-make test-localization-stress
mingw32-make test-slam-stress
mingw32-make localization-benchmark
mingw32-make slam-benchmark
```

Automated results are regression evidence. They are not desktop visual or
interaction acceptance.

## 11. Files Expected to Change

Production/UI integration:

- `include/app/Simulator.hpp`
- `src/app/Simulator.cpp`
- `include/ui/ApplicationLayout.hpp`
- `src/ui/ApplicationLayout.cpp`
- `include/ui/EditorToolbar.hpp`
- `src/ui/EditorToolbar.cpp`
- `include/ui/InspectorPanel.hpp`
- `src/ui/InspectorPanel.cpp`
- `include/localization/LocalizationVisualization.hpp`
- `src/localization/LocalizationVisualization.cpp`
- `include/slam/SlamVisualization.hpp`
- `src/slam/SlamVisualization.cpp`
- `Makefile` (new production directory and explicit runtime-test link objects)

Regression tests:

- `tests/SimulatorRuntimeTests.cpp`
- `tests/LocalizationSensorTests.cpp`
- `tests/SlamIntegrationTests.cpp`

Specification/documentation after decisions and implementation:

- `docs/specs/EditorUISpec.md` (currently missing/not started)
- `docs/specs/README.md`
- `Document.md`
- `README.md` if launch/controls change
- `docs/agent/STATUS.md` only after truthful final verification

The list is expected, not pre-authorized implementation scope. Recheck it after
the UI specification is approved.

## 12. New Files / Classes Expected

Recommended new production files:

- `include/visualization/RoboticsVisualizer.hpp`
- `src/visualization/RoboticsVisualizer.cpp`
  - `RoboticsVisualizer`
  - `RoboticsVisualizationData`
  - `RoboticsFollowSource` or equivalent presentation enum
- `include/ui/ViewportCamera.hpp`
- `src/ui/ViewportCamera.cpp`
  - `ViewportCamera`
- `include/ui/ViewControlBar.hpp`
- `src/ui/ViewControlBar.cpp`
  - `ApplicationViewMode`
  - `ViewControlBar`
- Optional small `include/ui/UiTextLayout.hpp` if one shared pixel-aligned text
  helper is needed by both toolbars and Inspector tabs.
- `scripts/ui_playtest.ps1` only after a working capture/observation path is
  chosen.

Do not add a generic widget framework, scene manager, event bus, render graph,
or symmetric `SimulationVisualizer` in this milestone.

## 13. Dependency / Architecture Risks

1. **Truth leakage:** passing `AMR`, `m_scanGroundTruthPose`, or an unlabeled
   truth transform into Robotics follow or LiDAR anchoring would violate the
   requested boundary.
2. **Frame conflation:** AMCL/world, odometry/world, path/world, and SLAM/local
   cannot be overlaid correctly without an explicit display transform policy.
3. **Reverse dependency:** `RoboticsVisualizer` must never call reset/update/plan
   APIs or expose callbacks into inference.
4. **Header coupling:** including `Simulator`, `AmclLocalizer`, or
   `SlamFrontend` in the Visualizer public header would make presentation depend
   on inference owners instead of result types.
5. **Event leakage:** Robotics clicks must not edit the map, and Inspector/top-bar
   clicks must not reach either viewport.
6. **Hidden-state resets:** view switching must not reset RNGs, sensor cadence,
   estimator state, path progress, or layer options.
7. **Camera contamination:** a shared view or follow flag would cause pan/zoom
   state to leak across views.
8. **Cache invalidation:** moving particle/SLAM geometry ownership can leave stale
   display data after reset, bootstrap, map revision, or option changes.
9. **Inspector measurement:** two-column wrapping can undercount content height,
   break independent scrolling, or overlap the footer at short heights.
10. **Typography false fix:** disabling smoothing or increasing size without DPI
    screenshots may trade blur for jagged glyphs or clipping.
11. **Makefile link gaps:** production wildcard discovery requires the new
    `visualization` directory in `SRC_DIRS`, and `SimulatorRuntimeTests.exe` has
    an explicit object list that must include every new linked UI/visualization
    object.
12. **Acceptance gap:** process survival or green tests cannot close the current
    screenshot/interaction acceptance gap.

## 14. Implementation Phases

No phase starts until the decisions in section 16 and an `EditorUISpec.md`
contract are approved.

### Phase 0 — Approve UI contract and baseline

- Resolve display frame, follow source/default, view-switch placement, and
  capture strategy.
- Write/approve `EditorUISpec.md` and update the spec index.
- Run and record the clean baseline before production edits.

Acceptance: approved requirements and zero unexplained baseline failures.

### Phase 1 — View state, layout, controls, and camera primitives

- Add `ApplicationViewMode`, top-bar/view-control geometry, `ViewControlBar`, and
  two independent `ViewportCamera` instances.
- Add pure state/geometry tests before changing render composition.

Acceptance: deterministic switching, event isolation, independent cameras,
resize/zoom/pan/follow state tests passing; existing render output unchanged.

### Phase 2 — RoboticsVisualizer boundary

- Add the read-only snapshot and `RoboticsVisualizer` composition.
- Move ownership of localization/SLAM presentation caches/options from
  `Simulator` into the Visualizer.
- Remove unnecessary inference-owner includes from presentation headers.

Acceptance: no forbidden dependency, no mutable domain input, observational
tests pass, and sensor/RNG/SLAM regression stays green.

### Phase 3 — Split render and input paths

- Make Simulation render only truth/editor/path/AMR content.
- Make Robotics render only approved robotics results.
- Gate pointer/editor input by active view while preserving continuous runtime.

Acceptance: switching views does not reset runtime; layer-policy tests pass;
desktop screenshots show no SLAM/AMCL/LiDAR overlays in Simulation view.

### Phase 4 — Inspector structured table

- Replace string bodies with rows plus multiline blocks.
- Preserve all existing fields, tabs, footer behavior, and scroll offsets.

Acceptance: long Reason/Message cases wrap without overlap, all scalar values
align in two columns, short-height behavior remains safe, and no information is
lost.

### Phase 5 — Toolbar and shared typography

- Implement bounds-aware integer-pixel centering.
- Run the character-size/smoothing/DPI screenshot matrix.
- Apply the accepted metrics to editor and view controls.

Acceptance: no clipping at supported widths and human approval of
`Select [S]`, `Obstacle [O]`, and all other labels at the agreed DPI cases.

### Phase 6 — Deterministic Camera Follow integration

- Wire Simulation truth target and approved Robotics estimated target.
- Implement pan-disable, re-enable/recenter, zoom preservation, and invalid
  estimate behavior.

Acceptance: deterministic camera-state tests pass and desktop playtest confirms
the target stays centered with unchanged zoom and no smooth damping.

### Phase 7 — Playtest loop and final verification

- Establish a working capture path, then execute the full real-desktop scenario.
- Fix concrete findings and replay from a clean process.
- Run the complete automated regression/stress/benchmark sequence.
- Update user documentation and `STATUS.md` with separate automated and desktop
  evidence.

Acceptance: zero functional regression failures, completed screenshot-backed
desktop checklist, human visual/interaction approval, and no unresolved
important architecture finding.

## 15. Verification / Acceptance Gates

### Architecture gate

- `RoboticsVisualizer` depends only on presentation/result types.
- No Robotics follow or LiDAR path reads `AMR` truth.
- SLAM display alignment remains labeled, one-way, and inference-neutral.
- View switch and visualization option changes do not affect sensor or inference
  sequences.

### Automated regression gate

- Targeted UI/camera/visualization tests pass after each phase.
- Normal regression reports 0 FAIL.
- Localization and SLAM stress suites report 0 functional failures.
- Benchmarks complete with finite results; timing changes are reported, not used
  as hardware-sensitive functional gates.

### Desktop visual gate

- Simulation and Robotics views are visibly distinct.
- Simulation contains no robotics estimation overlay.
- Robotics contains no truth AMR/editor overlay and clearly identifies display
  frame/follow source.
- Toolbar text is crisp and centered at approved window/DPI cases.
- Inspector rows align; long fields wrap; no clipping, overlap, or footer
  collision occurs.
- Active view, active tool, active tab, layer state, and Follow state are obvious.

### Desktop interaction gate

- Switch views repeatedly without reset or state loss.
- Place/select/delete/rotate/draw only in Simulation view.
- Pan and zoom independently in both views.
- Manual pan disables active-view Follow.
- Re-enable Follow and confirm immediate recenter with unchanged zoom.
- Confirm Simulation follows truth and Robotics follows only the approved
  estimate.
- Exercise Inspector tabs/scroll and relevant navigation/localization/SLAM
  controls.
- Capture before/after screenshots for every major state.

### Reporting gate

Final status must list separately:

- build result;
- targeted and full regression totals;
- stress/benchmark results;
- launch/process smoke result;
- screenshot-backed desktop visual result;
- real interaction result;
- remaining human-only or environment-blocked checks.

## 16. Open Questions / Decisions

1. **Robotics follow source:** recommend AMCL estimate as the first-version
   default because AMCL, odometry, and current path are already in the map/world
   display frame. Should SLAM follow be selectable now, or deferred until a
   dedicated SLAM-local view/frame selector exists?
2. **Follow default:** should Simulation start with Follow enabled, or preserve
   the current fixed camera until the user enables it? Recommendation: preserve
   current behavior by defaulting Follow off, while making the control visible.
3. **Robotics display context:** recommendation is no ground-truth AMR/editor
   overlay, with SLAM occupancy as the primary map layer and the existing
   truth-derived SLAM alignment explicitly labeled display-only. Should a known
   reference map layer be shown, and if so should it default off?
4. **LiDAR anchor in Robotics view:** recommendation is the selected follow/frame
   estimate (AMCL for the first version), with no truth fallback. Should odometry
   be an explicit user-selectable alternative when AMCL is invalid?
5. **View switch control:** recommendation is a visible segmented
   `Simulation | Robotics` control plus a separate Follow toggle in the top bar.
   Confirm whether a keyboard shortcut is also required.
6. **`Ctrl+0` scope:** recommendation is reset only the active camera and disable
   its Follow state. Confirm whether users expect both cameras to reset.
7. **Capture infrastructure:** should the next milestone first fix the current
   Windows Graphics Capture failure, or approve a narrow in-app framebuffer
   capture hook? Without one of these, desktop acceptance remains human-only.

