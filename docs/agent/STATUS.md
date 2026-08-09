# Project Status

## Current milestone

Approved `MapCoordinateSpec.md` conformance is complete for CoordinateMapper,
MapData runtime behavior, and map persistence.

## Completed behavior

- `CoordinateMapper` and `MapData` always retain a positive grid resolution.
- World-boundary containment uses half-open right and bottom edges.
- World-position obstacle insertion validates the input point; `GridCoord`
  insertion validates the corresponding cell center.
- Map loading strictly validates required records, field counts, declared
  obstacle/work-zone counts, and specification value constraints.
- Failed loads do not propagate parsing exceptions and leave existing map
  state unchanged.
- Existing first-version A* PathPlanner behavior remains unchanged.

## Verification

- Clean `mingw32-make all` passed on 2026-08-09.
- `mingw32-make test` passed all four suites: 43 PASS, 0 FAIL.
- `CoordinateMapperTests.exe`: 6 PASS, 0 FAIL.
- `MapDataTests.exe`: 15 PASS, 0 FAIL.
- `MapDataFileTests.exe`: 15 PASS, 0 FAIL.
- `PathPlannerTests.exe`: 7 PASS, 0 FAIL.

## Known limitations

- The Makefile does not track header dependencies, so a clean rebuild is
  required after header-only changes to avoid stale binaries.
- No approved `MapCoordinateSpec.md` failures remain.

## Next smallest step

Add Makefile header-dependency tracking as a separate, narrowly scoped build
reliability milestone.

## Important decisions

- Invalid CoordinateMapper construction falls back to the valid default
  resolution of `50.0f`; invalid setter input preserves the current value.
- World-position and `GridCoord` obstacle overloads retain their distinct
  boundary-validation semantics from `MAP-003`.
- Loading builds validated temporary state and assigns it only after the full
  file passes validation.
