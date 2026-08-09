# AGENTS.md

## Goal

Make small, correct, and reviewable changes.

Do not optimize for maximum implementation speed.
Prefer code that is easy to understand and maintain.

## Workflow

For every task:

1. Read the relevant specification under `docs/specs/`.
2. Inspect the related header, implementation, and tests.
3. Identify the smallest change needed.
4. Implement only that change.
5. Build or test when applicable.
6. If verification fails, inspect the cause and make the smallest fix.
7. When the requested task is complete, STOP and report.

## Scope

Only perform the explicitly requested task.

Do not:

- Implement the next milestone automatically.
- Modify unrelated files.
- Perform large refactors unless requested.
- Add new abstractions or dependencies unless necessary.
- Rewrite working code only for style.

If a task is too large, split it into smaller steps and implement only the current step.

## Source of Truth

Priority:

1. Approved specifications in `docs/specs/`
2. Explicit task requirements
3. Tests
4. Existing implementation

Do not invent important behavior when the specification is unclear.

## Architecture

Follow `docs/specs/SystemOverview.md`.

In particular:

- `Simulator` coordinates modules but does not contain path-planning algorithms.
- `MapData` owns persistent map state.
- Coordinate conversion goes through `CoordinateMapper`.
- `PathPlanner` must not depend on rendering or editor input.

## Code Style

Prefer straightforward C++.

Avoid unnecessary cleverness or abstraction.

When introducing a non-obvious C++ feature, STL container, or algorithm,
briefly explain why it is used.

## Report

After each task, report:

- Modified files
- What changed
- Verification result
- Remaining work

## Repository Handoff

`docs/agent/STATUS.md` is the shared project-state snapshot for ChatGPT,
Codex, and future development sessions.

Before starting a coding task:

1. Read `docs/agent/STATUS.md`.
2. Confirm that its current milestone matches the requested task.
3. Read the relevant specification and implementation.

After completing a task:

1. Update `docs/agent/STATUS.md` to reflect the actual repository state.
2. Record:
   - Current milestone
   - Completed behavior
   - Verification performed
   - Known limitations
   - Next smallest implementation step
   - Important decisions when relevant
3. Do not record planned behavior as completed.
4. Commit the implementation and status update together.
5. Push the completed commit to the repository.

`STATUS.md` is a current-state snapshot, not a chronological development log.
Keep it concise and replace stale information.