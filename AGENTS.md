# AGENTS.md

## Communication

- Use Traditional Chinese for explanations and reports.
- Before modifying files, briefly state:
  - Task goal
  - Relevant specification
  - Expected file changes
- After completing a task, report:
  - Modified files
  - Behavior changes
  - Test results
  - Unverified items
- Keep explanations clear and practical.
- Do not assume the project owner understands the internal implementation.

## Scope

- Only perform the requested task.
- Do not add unrelated features.
- Do not perform large refactors unless explicitly requested.
- Do not introduce new third-party libraries unless explicitly approved.
- Preserve behavior outside the requested scope.
- Keep the project buildable after each task.

## Project Direction

Current priority:

1. Specifications and tests for core modules
2. Map validation
3. A* path planning
4. Path rendering
5. Robot path following
6. Sensor simulation

Do not start later milestones before the current milestone is verified.

## Specifications

Approved specifications are stored under:

`docs/specs/`

The specification index is:

`docs/specs/README.md`

Before modifying a module:

1. Read the relevant specification.
2. Read related tests.
3. Inspect the relevant implementation.

Expected behavior priority:

1. Approved specification
2. Approved task acceptance criteria
3. Tests matching the approved specification
4. Existing implementation
5. Comments and historical documents

Existing code and tests are not automatically correct.

For a new feature or behavior change:

1. Add or update the relevant specification as Draft.
2. Define acceptance criteria and edge cases.
3. Wait for project-owner approval.
4. Add or update tests.
5. Implement the approved behavior.
6. Run automated tests and required manual checks.
7. Update the specification status.

Do not weaken or rewrite a specification only to make incorrect code pass.

## Tests

Automated tests are stored under:

`tests/`

Prefer automated tests for:

- Coordinate conversion
- Map data
- Save and load
- Map validation
- Path planning
- Other deterministic non-UI logic

Use manual acceptance tests for:

- UI layout
- Inspector scrolling
- Window resizing
- Grid appearance
- Mouse and keyboard interaction
- Rendering

Tests should reference specification requirement IDs when practical.

Do not:

- Delete or weaken failing tests without an approved requirement change.
- Report tests as passed without running them.
- Treat an expected failure as a completed feature.

## Bug Handling

When a bug is found:

1. Reproduce the problem.
2. Record the observed behavior.
3. Compare it with the specification.
4. Identify the affected requirement.
5. Add a regression test when practical.
6. Fix only the affected behavior.
7. Run relevant tests.
8. Report the root cause and verification result.

If expected behavior is missing or unclear, stop and request a decision before changing code.

## Task Interruption

Stop the task and report when:

- Required behavior is undefined.
- Specifications conflict.
- A large unrelated refactor is required.
- A new dependency is required.
- Save-file compatibility may be broken.
- Required tools or test environments are unavailable.
- An unrelated bug blocks progress.
- Continuing requires guessing an important design decision.

The interruption report must include:

- Current progress
- Blocking issue
- Affected files or requirements
- Available options
- Recommended option
- Files already modified

## Work Loop

For each coding task:

1. Understand
   - Confirm the goal and expected behavior.

2. Inspect
   - Read relevant specifications, tests, and implementation.

3. Plan
   - List expected file changes and test method.

4. Implement
   - Make only the requested changes.

5. Test
   - Build the project.
   - Run relevant automated tests.
   - Perform required manual checks.

6. Fix
   - Resolve build errors, test failures, and behavior mismatches.

7. Report
   - Summarize changes and verification results.

Do not stop after only writing code.

A task is complete only when relevant behavior has been verified, unless testing is impossible and the limitation is clearly reported.

## Report Format

After each task, report:

1. Summary
2. Relevant Specifications
3. Modified Files
4. Behavior Changes
5. Test Results
6. Manual Verification
7. Notes