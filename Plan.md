# Simulation / Robotics Visualization & Desktop UI Milestone

Status: Recommended implementation plan; implementation has not started.
Reviewed: 2026-09-05, local HEAD e29839ae5f8979abb98e1b38aed979a59b113da0.

This plan supersedes the previous Plan.md. It was formed from AGENTS.md,
STATUS.md, all existing specs, subsystem interfaces, runtime/render/input code,
and relevant tests before reading the previous plan. It records the recommended
architecture and execution order, not completed functionality or permission to
start implementation in this planning turn.

## 1. Scope and verified baseline

This turn changes only Plan.md. Production code, tests, Makefile, STATUS.md,
and the user's existing AGENTS.md modification remain untouched. No build,
application launch, desktop playtest, commit, push, or remote synchronization was
performed. Historical test totals below are read from STATUS.md, not rerun.

The next implementation milestone covers:

1. Separate Simulation presentation from robotics estimation presentation.
2. Embed an independent RoboticsVisualizer subsystem in the existing window.
3. Provide explicit fixed-frame selection, native-frame layers and availability.
4. Remove SLAM, AMCL, odometry and robotics scan overlays from Simulation.
5. Convert Inspector content to sections with two-column property tables.
6. Correct Toolbar typography and pixel alignment, then inspect desktop clarity.
7. Add Simulation Follow ON by default and pan-to-disable behavior.
8. Maintain independent Simulation and Robotics camera state.
9. Establish a repeatable real desktop observe-act-verify workflow.
10. Require desktop acceptance separately from automated regression.

Keep the single-threaded SFML loop and existing inference, sensor cadence, RNG
fan-out, navigation modes, collision rollback and persistence behavior. Exclude
ROS integration, a general TF server, asynchronous rendering, docking/pop-out
windows, a widget framework, new SLAM algorithms, estimator fusion, map
registration, SLAM-based navigation and camera smoothing. Do not create a
SimulationVisualizer solely for naming symmetry. Shared sensor DTO relocation
out of localization/ is also outside this milestone.

### Source-backed findings

| Area | Current implementation and implication |
|---|---|
| Application | src/main.cpp constructs Simulator. Simulator::run performs processEvents, update(dt), render. There is one simulation camera plus a pixel-space UI view. |
| Truth/editor | Environment owns MapData and editor tool state. Simulator owns AMR, selection, validation and path execution. Keep these authorities. |
| Motion/sensors | Simulator::update rolls back collisions before updateLocalization acquires one immutable OdometryDelta + LaserScan pair. AMCL and SLAM share measurements with separate RNG streams. |
| AMCL | AmclLocalizer accumulates odometry; particles and estimate are committed when updateWithScan succeeds. The latest estimate can predate the latest acquired scan. Initialization may provide an estimate before a scan update. |
| SLAM | SlamFrontend owns its local grid and bootstraps identity at the first informative scan. resetSlamForCurrentPose and bootstrap currently set a truth-derived display origin. |
| Rendering | Simulator::render draws Environment, SLAM, truth-anchored scan, AMCL/odometry, path, then truth AMR on one canvas. Leaf renderers already exist but Simulator owns their options/caches. |
| Coordinates | MapCoordinateSpec and SlamV1Spec use x-right, y-down and the existing yaw convention. SLAM cell conversion is relative to its own grid origin, independent of CoordinateMapper. |
| Odometry | OdometrySimulator::reset seeds its accumulated pose with the supplied truth pose. Kidnap rebase preserves that accumulated belief. It is not an independently published ROS odom frame. |
| Cache | SlamVisualization bakes displayOrigin into cell vertices but keys its cache only by map revision. SlamOccupancyGrid::reset returns revision to zero. Frame/session changes need explicit cache handling. |
| UI | ApplicationLayout owns three rectangles. EditorToolbar owns seven buttons and hit testing. Inspector owns four tabs and separate scroll offsets; section bodies are newline strings. |
| Text | Toolbar centers using text width only, omits local-bounds position, uses fixed vertical +8 and fractional button widths. These are source findings, not a verified diagnosis of every blur cause. |
| Events | Hotkeys run before pointer routing. Pan is a single boolean. Held driving keys are polled without an application-focus guard. New view routing must address capture and focus explicitly. |
| Verification | Runtime tests inspect headless seams, geometry and state, not a desktop interaction loop. STATUS records build, 226 normal passes, 4 localization stress passes and 5 SLAM stress passes; manual UI acceptance remains outstanding. |

The specs index is incomplete: it omits the existing SlamV1Spec and still marks
some implemented test coverage as not created. EditorUISpec is not present.
PathPlanner has clearance and explicit-pose overloads beyond the original
one-argument spec; preserve them, without extending or redesigning navigation.
During a later authorized specification phase, document this milestone's
contracts without rewriting unrelated specifications.

## 2. Architecture and ownership

Recommended composition:

    Simulator
      lifecycle / runtime commands / accepted motion / sensor fan-out
      presentation adapter: coherent read-only observations
      ApplicationLayout + view headers: rectangles / hit targets / focus
      Simulation camera
      Simulation render path: Environment + path + AMR + selection
      RoboticsVisualizer
        Robotics camera state per fixed frame
        native-frame layer options / status / caches
        LocalizationVisualization + SlamVisualization + reference/path layers
      InspectorPanel: structured diagnostic sections / tables / scrolling

### RoboticsVisualizer boundary

Add include/visualization/ and src/visualization/. RoboticsVisualizer belongs here
because it presents multiple subsystems' results. It owns robotics draw order,
layer options, geometry caches, frame availability and its camera. Simulator
owns the Simulation camera and delegates bounded pointer/camera actions to the
RoboticsVisualizer; it does not reach into individual robotics layer renderers.

Use a small concrete API: supply read-only presentation data, assign viewport,
handle camera/layer actions, obtain display status, draw to an sf::RenderTarget.
Simulator remains responsible for window lifecycle, global UI composition and
dispatching runtime commands. View widgets emit typed actions; the Visualizer
cannot reset an estimator or execute a robot command.

Robotics inputs may contain result types, const particle/grid views, known-map
geometry, path geometry, observation stamps and paired scan anchors. They must
not contain AMR, Environment, Simulator, estimator owners, sensor simulators,
mutable domain references, RNGs or callbacks into runtime/inference. Replace
SlamVisualization.hpp's unnecessary SlamFrontend.hpp include with actual
result/grid dependencies.

The application adapter is the only place that reads multiple owners to assemble
presentation data. Do not move acquisition, SLAM prediction, AMCL cadence or
navigation decisions into presentation. The adapter may format provenance and
track observation identity; it does not compute a new pose estimate.

Retain existing leaf renderers in their directories initially, adapting their
interfaces as needed. A mass file move is not necessary for this boundary.

### Data lifetime and update ordering

Use one frame-scoped read-only input aggregate. Small metadata/poses are values;
large arrays/grids may be borrowed only during the synchronous presentation
update. A renderer must not retain those references across a subsequent domain
update/reset. Retained scan samples are owned bounded copies; retained geometry
is renderer-owned. No full map copy every render frame and no new threading.

Order each iteration as:

1. Route window/UI/input events and execute authorized runtime commands.
2. Apply motion, collision rollback and existing sensor/estimator updates.
3. Publish coherent presentation records and refresh changed geometry.
4. Update Follow from accepted/published poses and refresh cursor conversion.
5. Render Simulation and Robotics inside their bounds, then pixel-space UI.

A window/view/frame/layer operation changes presentation only. It must not reset
or pause inference, regenerate measurements, consume RNG, or change path
progress. Real wall-clock dt may vary with rendering cost; deterministic
invariance checks must compare identical accepted motions/measurement inputs,
not assume two live runs have identical timing.

## 3. Fixed frames: native estimates first

### Coordinate contract

A fixed frame is the reference in which layer coordinates are interpreted.
Camera center/zoom/follow controls where that reference is viewed; moving a
camera does not transform the data into a new frame. A grid's origin offset is
also distinct from an inter-frame transform.

Retain project world units and x-right/y-down coordinates. Positive yaw follows
the existing cos/sin convention and appears clockwise on screen. Do not relabel
distances as meters or change to ROS axes in this UI milestone.

The mini-RViz concept means explicit frame identity, independent display layers,
per-layer availability/provenance and a camera. It does not mean the repository
already implements ROS frame semantics. REP 105 distinguishes map corrections
from continuous drifting odometry; the current accumulated odometry pose alone
does not implement that transform tree.
Reference: [REP 105, upstream source](https://raw.githubusercontent.com/ros-infrastructure/rep/master/rep-0105.rst).

| Frame / coordinate domain | Authority and lifetime | First-version use |
|---|---|---|
| sim_world | MapData coordinates; AMR truth expressed here | Simulation only, plus explicitly labeled diagnostic values in Inspector |
| map | AMCL known-map coordinates, currently supplied by the same MapData | Robotics fixed frame for known map, AMCL and existing planned path |
| slam_local | SLAM's first informative scan establishes identity; a new SLAM reset creates a new epoch | Default Robotics fixed frame for estimated occupancy and SLAM poses |
| base / laser | Measurement-local geometry; scan carries base-to-sensor x/y/yaw extrinsics | Compose with the observation's matching estimated base pose |
| legacy world-seeded odometry | Accumulated odometry initialized in the current map/world coordinate chart | Optional map-mode diagnostic marker only; not an odom fixed-frame provider |

The current sim_world-to-map coordinate correspondence is identity because AMCL
uses that exact map representation. This says nothing about AMCL pose accuracy.
It must not be generalized to imported maps or SLAM maps. A known-map layer is
explicitly labeled “Known map (simulator-provided)” and is never copied into SLAM.

### First-version fixed-frame selector

Provide a visible selector with exactly two choices:

- SLAM local: default. Render occupancy and valid SLAM pose/prediction natively.
- Map: render known reference map, AMCL particles/estimate/covariance and optional
  existing navigation path in their native map coordinates.

AMCL initialization is not required to select Map or view its reference map.
SLAM local before bootstrap shows axes and “Waiting for informative scan”; do
not invent an identity robot pose from a default-constructed result.

Keep layer preferences per frame. Incompatible layers are visibly unavailable,
with a reason such as “No map ↔ slam_local transform”; they are not silently
drawn at identity. Changing an Inspector tab does not select a fixed frame.

### Deliberately absent cross-estimator transform

There is no estimated map-to-slam_local registration in the current project.
Do not reuse m_slamDisplayOrigin in the new default Robotics pipeline. The SLAM
spec permits truth for labeled test/display diagnostics; it does not require
truth-aligned rendering or establish an estimated frame relationship.

Do not recompute a transform each frame from AMCL pose and SLAM pose merely to
make their robot markers coincide. That would absorb their disagreement and move
the SLAM grid as estimates change. Do not use camera Follow source to select a
scan anchor, establish a fixed frame, or align maps.

Truth-aligned cross-frame comparison is deferred from the first version. If
requested later, it needs a separate explicit diagnostic mode with transform
source, direction, captured observation, session identity and visible truth
provenance. It must remain outside inference and ordinary Robotics Follow.
A genuine estimated registration is a separate algorithm milestone.

### Scan geometry and temporal pairing

For a valid observation at sequence k, with fixed frame F:

    p_F = T_F_base(k) * T_base_laser(k) * p_laser(k)

The base pose, scan and extrinsics must refer to that observation. Apply
extrinsics exactly once. Render subsampling stays independent from AMCL selected
beams and SLAM matching beams. Invalid ranges do not render as hits; max-range
rays may be shown clamped to range, but never as occupied endpoints.

- SLAM local: pair each processed scan with that SlamUpdateResult. A rejected
  match is labeled prediction-only, not corrected. Lost/invalid states remain
  explicit; suppress scan geometry if no usable same-observation pose exists.
- Map: retain the scan plus final AMCL estimate from the same successful
  updateWithScan dispatch. Between AMCL updates show the last paired observation
  with its age/sequence. Never attach the current m_currentScan to an older AMCL
  estimate. Initialization alone does not supply a paired scan.
- Prediction-only/all-max AMCL commits can retain rays with weak-observation
  status; a failed commit does not relabel an older observation as current.
- No truth, legacy odometry, or other estimator fallback when a pair is missing.
  Do not force more AMCL updates or implement display-side extrapolation.
- Camera Follow remains independent of scan visibility and scan source.

## 4. Presentation records, invalidation and layers

Add presentation metadata at the application boundary rather than changing
sensor measurements or inference algorithms:

- monotonically increasing acquisition sequence and simulation-time stamp;
- map session/generation plus geometry revision;
- SLAM epoch plus map revision;
- AMCL initialization epoch and committed-observation sequence;
- layer frame identity, data validity and observation quality/age.

Do not use sensorUpdateCount alone as a timestamp: not every committed AMCL
update increments it. Use dispatch.amclUpdated and the acquisition sequence.
Clear paired samples on the relevant initialization/reset; keep at most the
latest usable pair per estimator. Layer status distinguishes Disabled, Ready,
Waiting for data, No transform and Stale/Degraded.

Cache SLAM vertices in slam_local, without a baked truth transform. Key occupancy
by epoch + revision + grid configuration identity. Known-map geometry also needs
map identity/generation, not revision alone. Particle geometry refreshes on AMCL
initialization/commit; turning a hidden layer back on must use current data.
Camera changes must not rebuild full map geometry.

| Event | Required presentation handling |
|---|---|
| Pan, zoom, focus, frame/layout change | Preserve domain state and layer preferences; select only matching frame data/camera |
| SLAM-only reset | Clear SLAM pair/cache, increment SLAM epoch, invalidate its camera frame; preserve AMCL/map/Simulation state |
| Delayed SLAM bootstrap | Establish the new local frame at that informative observation, not at reset time |
| AMCL local/global reset | Increment AMCL epoch, clear its scan pair/particles until republished; preserve SLAM and its camera |
| Robot reset / configured Start synchronization | Mirror existing runtime reset effects; publish all affected epochs, never replay pre-reset samples |
| Successful map load or clear | New map generation, clear invalid path/pairs/caches according to existing resets; no samples from the previous map |
| Map geometry edit | Rebuild reference cache and mark older AMCL scan context stale until refreshed; preserve current inference policy and SLAM session |
| Kidnap | Preserve existing odometry-rebase/estimator behavior; invalidate pre-teleport scan presentation until fresh processing; never realign SLAM with truth |
| Lost / invalid estimate | Preserve map as appropriate, show quality/reason, suspend Follow target; never show stale data as a new valid correction |

Map load failure must leave presentation and runtime sessions intact. Coordinate
discontinuities reset only the affected camera context; ordinary geometry
revisions do not recenter cameras.

### Layer allocation and defaults

| Layer | Simulation | Robotics: SLAM local | Robotics: Map |
|---|---|---|---|
| Editable map, work zones, Start/Goal, cursor, selection | Yes | No | No editor affordances |
| Ground-truth AMR | Yes | No | No |
| Executed/planned path | Yes | Unavailable | Optional OFF, labeled current navigation result |
| SLAM occupancy / pose / predicted pose | No | ON, validity/status aware | Unavailable |
| Known reference map | Via Environment | Unavailable | ON, simulator-provided label |
| AMCL particles / estimate / covariance | No | Unavailable | ON, validity/status aware |
| Paired LiDAR rays / hit points | No | OFF / OFF, SLAM pair | OFF / OFF, AMCL pair |
| Legacy world-seeded odometry | No | Unavailable | OFF, diagnostic provenance label |

Retain F1/F2/F3/F4/F6/F7/F8 as shortcuts for their documented layer/diagnostic
actions, routed to Robotics options even while Simulation has focus. An
incompatible layer reports unavailable without switching frames. Expose clickable
layer controls so the workflow is discoverable; SLAM layers also have visible
toggles. Frame axes, legend and status must distinguish free/occupied/unknown,
estimator source and prediction/correction, without relying only on color.

## 5. Embedded layout and event routing

Recommendation: show Simulation and Robotics side by side at the default
1440×900 size, with the shared 360-pixel Inspector on the right. This supports
driving while observing estimation without repeated view switching.

Use a simple fixed split, not a draggable docking system. Both panes have a
pixel-space header with name, focus indication and camera controls; Robotics
also exposes Fixed Frame. EditorToolbar stays above the workspace.

Calculate every rectangle in ApplicationLayout. As an initial contract, require
at least 400 pixels per pane plus an 8-pixel divider for split mode. Below that
workspace width, use an explicit Simulation / Robotics tab to show one pane at
a time. Preserve both subsystem instances and all camera state. At 1024×720 this
normally becomes the tabbed layout; do not squeeze text or change world units.
Confirm these product thresholds during the desktop contract review.

Pointer and keyboard rules:

1. Handle close/resize/focus/capture cancellation before ordinary commands.
2. Resolve headers, Toolbar and Inspector in UI pixel coordinates first.
3. Route a pointer action to exactly one viewport; blank UI space consumes input.
4. Convert pixels with the owning viewport's camera and rectangle only.
5. Record gesture owner on press. Release outside a viewport ends that owner's
   gesture; it cannot create an editor action in another viewport.
6. Losing focus, Escape, hidden-pane transition or incompatible layout/frame
   change cancels active gestures. A work-zone drag released outside Simulation
   is canceled rather than committed at an unrelated coordinate.
7. Middle-button drag pans either view. Existing Pan tool + left drag remains
   available in Simulation. Robotics left clicks cannot edit or select truth.
8. Wheel over a viewport zooms that camera; wheel over Inspector body scrolls;
   wheel over headers/Toolbar does not leak into a viewport.
9. Editor tools, Delete and Q/E object rotation require Simulation focus.
   Clicking an editor tool deliberately focuses Simulation. Runtime movement
   keys and existing planning/reset commands work with either viewport focused,
   but are suppressed while the app is unfocused or Inspector/UI owns focus.
10. Kidnap-at-cursor requires a current valid Simulation cursor, never a stale
    Robotics or Inspector coordinate. Navigation mode remains unrelated to
    Robotics fixed-frame choice.

Keep automatic path execution behavior when focus changes; this milestone adds
input gating, not a new simulation pause feature. Refresh hover coordinates after
Follow/resize/pan so preview and click conversion use the displayed camera.

Render each pane with its assigned viewport and explicit clipping, then restore
UI view state. Draw a bounded background per pane; clearing the entire window
for the second pane must not erase the first. Handle zero-sized/minimized
viewports without divide-by-zero or invalid SFML views.

## 6. Camera contract

Use a small ViewportCamera holding center, world-units-per-pixel zoom, viewport,
Follow state and drag state. It depends on an optional pose/target, not AMR or
estimator classes. Reuse the class without sharing its instances.

- Simulation starts Follow ON, targets the accepted post-collision truth pose.
- Robotics starts Follow OFF. In SLAM local its only follow source is a valid
  Tracking SLAM pose; in Map it is a valid AMCL estimate with its quality shown.
  AMCL ambiguity/recovery or invalid data suspends the target. Valid Tracking
  estimates may be followed without claiming navigation-grade convergence.
- Follow states are Off, Following and Waiting. Waiting freezes the center and
  shows why. Returning to a usable estimate resumes the user's enabled Follow.
- Re-enable Follow: center immediately on the current usable target, preserve
  zoom. Follow affects translation only; heading does not rotate the camera.
- The first nonzero manual drag disables Follow only for that gesture's camera.
  A click without movement does not disable it. No automatic re-enable on release.
- Wheel zoom preserves Follow; use center-based zoom with bounded finite scale.
  Initial limits: 0.1–20 project world units per physical viewport pixel.
- Follow has no damping/interpolation. AMCL corrections may visibly jump; do not
  hide estimator behavior with camera smoothing.
- Resize preserves center, world-units-per-pixel and Follow state, recalculating
  view size from the new viewport dimensions.
- Ctrl+0 and the header Reset action affect only the focused camera: restore
  default scale (1 unit/pixel), disable Follow and use that frame's default
  center. Simulation uses configured Start or the existing default robot
  position; SLAM local uses (0,0); Map uses known-map boundary center.
- Store a separate Robotics camera context for map and slam_local so switching
  fixed frame cannot reinterpret the same numeric center in a different frame.
  Restore a saved context only while its frame epoch is still valid.
- A new SLAM epoch resets only its center/scale context to local defaults and
  preserves Follow intent as Waiting until bootstrap. A new map generation
  invalidates only the corresponding map camera context.
- In tabbed layout, hidden cameras retain settings; enabled Follow can update
  against valid targets before the pane is shown again. Visibility never resets
  or gates simulation/estimator updates.

## 7. Inspector: sections and two-column tables

Preserve Map, Navigation, Localization and SLAM tabs, independent scrolling and
the short-height footer fallback. Replace newline-delimited section bodies with
a small ordered presentation model:

    Section: title + ordered entries
    PropertyRow: label + value + optional semantic status
    TextBlock: label + wrapped text + optional semantic status

Use one label/value divider for the panel, based on available width, not on
changing numeric values. Left-align values; include explicit units and frame
labels. Keep scalar values in rows and long explanations in full-width blocks.
Represent entries in one ordered sequence so narrative blocks can occur beside
the relevant properties rather than all at the bottom.

Measure rows and blocks before clamping scroll and drawing. Wrap both columns as
needed; row height is the maximum of their measured heights. Long unbroken
tokens must wrap or have an explicit readable overflow treatment. Clip the body;
headers/tabs/footer remain fixed and do not overlap. Preserve fields, controls,
validation messages, application status and navigation stop reason.

Group frame/epoch, scan age, layer availability and Follow status with the
relevant diagnostics. Show native SLAM coordinates and native AMCL map
coordinates explicitly. Existing truth/error diagnostics remain clearly labeled
in Inspector, outside the normal Robotics snapshot and camera path. Hide or
mark cross-map/time-mismatched diagnostics unavailable instead of reporting a
misleading error.

The Inspector formats supplied state. It must not validate maps, choose
navigation gates, estimate poses, own layer truth, or become another map canvas.

## 8. Toolbar typography and pixel alignment

Correct measurable placement first, then determine actual raster quality on the
desktop. The source alone cannot identify OS bitmap scaling, fallback font,
smoothing and subpixel placement contributions.

- Draw UI at the actual render-target pixel size, outside world cameras.
- Derive button edges from cumulative rounded positions so proportional sizing
  cannot accumulate gaps or overlap. Drawing and hit testing share rectangles.
- Center using full glyph local bounds: subtract bounds.position and account
  for bounds.size, then round final UI placement. Use a consistent visual
  baseline/height policy for neighboring labels.
- Do not scale a rendered text texture to make labels fit. Use integer character
  sizes and measure them in the actual selected font.
- Evaluate 14/15/16 pixel sizes, beginning with 16 for readability, at default
  and compact widths. Prefer compact documented labels over smaller text.
- Share only the small text-placement helper needed by Toolbar, view headers
  and Inspector tabs; do not introduce a general UI abstraction.
- Record the loaded font and render-target/client size in playtest evidence.
  Compare smoothing settings only if the capture shows a problem. Do not assume
  disabling smoothing fixes blur.
- Inspect native-size captures and the real display at the user's scaling.
  Target 100%, 125%, 150% where available; record unavailable cases. Any global
  DPI change remains a manual tester action. If OS bitmap scaling is confirmed,
  address the narrow DPI cause in the UI task rather than compensating with
  arbitrary world-view scaling.

## 9. Real desktop playtest workflow

Establish observation before substantial UI implementation. The old plan's
capture error and claims about installed tools are historical observations,
not verified current environment facts. No capture facility was tested here.

The implementation preflight must establish: correct executable/process,
working directory, actual client dimensions, focus, readable current screenshot,
input delivery and a fresh post-action screenshot. Prefer supported native
desktop tooling. If unavailable, have a human operate and capture the real app;
do not convert missing desktop evidence into a passing automated acceptance.

A framebuffer dump is useful rendering evidence but does not prove focus,
OS-DPI behavior, native input delivery or interaction feel. Do not add a capture
hook or repair host tooling automatically as part of this milestone. Expand that
scope only if the operator chooses it; the human-led desktop route is sufficient.

Later add a narrow scripts/ui_playtest.ps1 launcher/checklist helper if useful:
build as requested, launch one known process from repository cwd, record readiness
and run metadata, and stop only that owned process during cleanup. A launcher
cannot mark UI scenarios passed. Use a temporary scenario directory/map copy;
do not overwrite the user's saved map/config. Screenshots and logs go to an
external artifact directory with a run manifest.

Each case records preconditions, real input actions, expected behavior, observed
behavior, before/after screenshots (or a short recording for motion), PASS/FAIL/
BLOCKED/NOT RUN, and issue/retest evidence.

| Case | Real desktop actions and acceptance |
|---|---|
| D01 Startup | Inspect both panes at 1440×900: Simulation Follow ON, Robotics SLAM local, separate frame labels, no estimation overlays on Simulation. |
| D02 Fixed frame | Bootstrap away from world origin with nonzero heading; switch SLAM local ↔ Map. SLAM begins at identity, AMCL stays in map, missing-transform layers report unavailable. |
| D03 Layers/scan | Toggle every layer by visible control and applicable shortcut; move/rotate with nonzero LiDAR extrinsics; inspect SLAM and AMCL paired-scan status including slower AMCL updates. No truth fallback or double extrinsic. |
| D04 Follow | Drive and rotate; zoom while following; pan until Follow turns OFF; release and re-enable. Check immediate recenter, unchanged zoom and independent cameras. |
| D05 Invalid estimates | Before bootstrap, after AMCL reset, and in available Lost/Ambiguous/Recovering scenarios, inspect unavailable/degraded labels and Waiting Follow. Never follow truth in Robotics. |
| D06 Routing/capture | Click both panes, blank headers and Inspector; drag across pane/Inspector edges, release outside, Alt-Tab and return. No editor leakage, stuck pan or unintended held-key motion. |
| D07 Inspector | Inspect all four tabs, every field, long Reason/Message/validation text, independent scrolling and compact-height footer. No clipped values or shifting columns. |
| D08 Typography/resize | Inspect all Toolbar/header/tab labels at 1440×900, 1280×800 and 1024×720, maximize/restore and available DPI cases. Check native-size clarity, hit targets and split/tab transition. |
| D09 Lifecycle | SLAM-only reset, AMCL-only/global reset, robot reset, Start change, map load/clear and kidnap on disposable data. No previous-epoch scan/map artifacts; unrelated state survives. |
| D10 Existing flows | Place/select/delete/rotate, draw/cancel a work zone, plan/execute in both navigation modes. Camera/frame choices do not change command semantics. |

If a state cannot be reached reliably through available UI, record that case as
NOT RUN and arrange a human/reproducible scenario; do not fabricate completion
from a headless fixture. Baseline capture precedes edits, and defects follow
fix → rebuild → fresh process → replay. A human explicitly accepts text clarity,
layout readability, camera feel and discoverability.

## 10. Automated regression and completion gates

Automated checks establish invariants, not UI acceptance. Extend existing suites
or add a focused camera/frame/table suite only where independent pure logic
benefits. Avoid assertions that simply mirror a draw-policy enum.

Required targeted coverage:

- Nonzero translation/yaw scan composition, negative coordinates, extrinsics
  exactly once, physical-hit/max-range rules and frame incompatibility.
- Native SLAM identity after delayed bootstrap; no truth inputs required by
  default Robotics data; same input results unaffected by display actions.
- AMCL paired scan from its committed sequence, including all-max commits,
  initialization without scan and failed commits; no mixed epochs.
- Same map revision across reset/replacement cannot reuse stale geometry.
- Separate Simulation and per-frame Robotics camera contexts; Follow defaults,
  first-motion pan disable, zero-motion click, zoom/resize, invalid target and
  active-camera reset.
- Actual input-routing logic for header/Inspector consumption, capture owner,
  out-of-viewport release, focus loss and compact layout changes.
- Table wrapping/content measurement before scroll clamp; independent tabs,
  long words, empty values, compact footer and glyph-bound containment.
- Integration retains sensor fan-out, RNG isolation, independent estimator
  reset and navigation gate behavior.

Use existing SimulatorRuntimeTests, LocalizationSensorTests and
SlamIntegrationTests where relevant. Preserve geometric display-transform
coverage if its API changes by testing the replacement composition helper;
do not retain a truth-aligned production path just to preserve a test call.

After targeted tests, build the app and run normal regression. At the integrated
milestone gate run a clean build and both stress suites:

    mingw32-make clean
    mingw32-make all
    mingw32-make test
    mingw32-make test-localization-stress
    mingw32-make test-slam-stress

Run localization-benchmark and slam-benchmark if sensor dispatch/inference
integration or measured runtime performance is affected. Report changed timing
rather than weakening functional gates. Existing estimator benchmarks do not
measure two-pane UI responsiveness; inspect that during desktop playtesting.
The Makefile lacks generated header-dependency tracking, so a clean final build
is necessary after header changes.

Completion requires all of:

- Architecture/frame/lifetime review passes with no implicit cross-frame bridge.
- Targeted and normal regression pass; both stress suites have no unexplained
  functional failures. Unrelated baseline failures are reported, not broadened
  into this UI milestone.
- D01–D10 have real desktop evidence for the agreed acceptance matrix.
- A human accepts visual clarity and interaction; material issues are fixed and
  replayed. BLOCKED or NOT RUN acceptance cases remain incomplete.
- Report build, regression, stress/performance, launch smoke, desktop visual,
  desktop interaction and human acceptance separately.
- Update STATUS only in the later implementation turn, with actual evidence.
  Never equate green tests or process survival with UI acceptance.

## 11. Implementation sequence and expected files

No implementation is performed by this planning task. A later explicit
implementation task uses this sequence; each stage stays inside this milestone.

| Stage | Work and likely files | Exit evidence |
|---|---|---|
| 0 Contract + desktop baseline | Resolve section 13 choices; create docs/specs/EditorUISpec.md and update the relevant index entries; establish capture/operator and repeatable scenario. Read approved specs before changes. | Concrete UI/frame contract, baseline build/regression and a real observed desktop session; failures clearly identified. |
| 1 Frame/data boundary | visualization/RoboticsVisualizer and data records; adapt localization/LocalizationVisualization and slam/SlamVisualization; app/Simulator presentation adapter and epoch/pair bookkeeping. | Native-frame and temporal tests, correct cache lifetimes; no estimator algorithm change. |
| 2 Cameras/layout/input + render split | ui/ViewportCamera, small view header/control component, ApplicationLayout; app/Simulator routing/render changes. Wire Follow here, not as a late retrofit. | Targeted routing/camera regression and early D01–D06/D09 desktop evidence; Simulation overlays removed. |
| 3 Inspector and typography | ui/InspectorPanel, EditorToolbar, optional UiTextLayout helper shared with headers. | Table/geometry checks plus D07/D08 visual review and fixes. |
| 4 Integrated acceptance | Optional scripts/ui_playtest.ps1, full D01–D10 replay, build/regression/stress, applicable benchmarks, control documentation. | Separate automated and human desktop gates; truthful STATUS update after verification. |

New production paths are expected to be limited to visualization/RoboticsVisualizer
(and a data header only if needed), ui/ViewportCamera and a small view-controls
component. Existing rendering/Inspector files adapt in place. Makefile changes
in the future implementation must add visualization to SRC_DIRS and update every
affected explicit test link list; production wildcard discovery alone is
insufficient. Keep include/ as the sole project include root and tests flat.

Do not automatically add dependencies, generated artifacts, a general
FrameManager, event bus, TF history service or a new simulator architecture.
No phase implies commit/push authorization.

## 12. Changes from the previous plan

| Previous recommendation or omission | Current decision and reason |
|---|---|
| Robotics fixed in map/world with truth-aligned SLAM | Native slam_local / map selector; no default bridge. A display permission in SlamV1Spec is not a requirement to truth-align estimates. |
| AMCL default Follow and scan anchor selected with Follow source | Fixed-frame-specific Follow; scan paired with its own estimator observation independently of camera/layer options. |
| Simulation Follow default OFF left as a question | ON is required by this task and is settled. |
| One visible Simulation/Robotics mode | Default embedded side-by-side panes, compact-width tabs as an explicit fallback; simpler simultaneous observation. |
| One Robotics center across potential frames | Frame-specific camera contexts and epoch invalidation prevent coordinate reinterpretation. |
| Broad “world” label for odometry and path | Known-map provenance, world-seeded odometry diagnostic, explicit native layer compatibility; no invented ROS odom tree. |
| Latest scan plus optional estimated anchor | Sequence/epoch-paired observations; AMCL cadence and stale data are part of the contract. |
| Revision-only caches and generic reset risks | Native geometry, session + revision keys, reset/bootstrap/kidnap handling matrix. |
| Capture repair/in-app hook central to closing acceptance | Revalidate actual capability first; real human desktop route is valid. Framebuffer output alone cannot certify desktop input/DPI. |
| Capture tooling availability/error stated as current | Historical, unverified claims removed from the current baseline. |
| Follow integration and playtest late in the phases | Follow is integrated with cameras; observe the desktop before UI work and replay during each visible stage. |
| Repeated approval of unspecified implementation details | Concrete recommended defaults; only product/environment choices remain open. |

Retained useful parts: a cross-subsystem Visualizer, observational leaf renderers,
no symmetric SimulationVisualizer, four Inspector tabs, structured rows and long
blocks, glyph-bounds alignment, deterministic regression and separate human
acceptance.

## 13. Human decisions still open

These are bounded product/acceptance choices, not permission to implement now.
The recommendations above form a complete default plan; change only the affected
contract if the user chooses otherwise.

1. Embedded presentation: accept side-by-side at adequate width with compact
   tabs, or prefer always-tabbed views? Recommendation: side-by-side plus fallback.
2. Initial Robotics focus: keep SLAM local as default (recommended for inspecting
   the independently built map), or start in Map for AMCL-focused work? Both
   fixed frames are part of V1 regardless.
3. Acceptance environment: name the desktop operator and available Windows
   scaling/client-size matrix. Recommendation: native desktop capture and human
   acceptance; use a human-led session if automation cannot observe the window.
   Unavailable required cases must remain pending or be explicitly removed from
   the agreed matrix.
4. Cross-frame comparison: is truth-aligned comparison required in this same
   milestone? Recommendation: defer it. If required, define a separate labeled
   diagnostic contract before expanding implementation; do not reinstate the old
   implicit world-aligned Robotics design.

Simulation Follow ON, independent cameras, no estimation overlays in Simulation,
sensor-only SLAM, and automated-tests-not-UI-acceptance are settled requirements,
not open questions.
