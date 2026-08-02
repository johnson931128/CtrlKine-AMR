Status: Approved
Last Updated: 2026-08-02
# Map and Coordinate Specification

## 1. Scope

This specification defines the required behavior of:

* GridCoord
* CoordinateMapper
* MapData
* Map save and load

Rendering, editor interaction, validation, and path planning are outside this specification.

## 2. Coordinate Definitions

### COORD-001 Grid coordinate axes

A GridCoord shall contain:

* `col`: horizontal grid index
* `row`: vertical grid index

Increasing `col` moves right.

Increasing `row` moves down in the current SFML world-coordinate convention.

### COORD-002 World-to-grid conversion

World coordinates shall be converted using the configured grid resolution.

For a grid resolution of 50:

* World `(0, 0)` maps to grid `(0, 0)`.
* World `(49.9, 49.9)` maps to grid `(0, 0)`.
* World `(50, 50)` maps to grid `(1, 1)`.
* World `(-0.1, -0.1)` maps to grid `(-1, -1)`.

Conversion shall use floor semantics, including for negative coordinates.

### COORD-003 Grid top-left conversion

The top-left world coordinate of a grid cell shall be:

* `x = col × grid resolution`
* `y = row × grid resolution`

### COORD-004 Grid center conversion

The center of a grid cell shall be half a grid resolution from its top-left corner on both axes.

### COORD-005 Grid resolution validity

Grid resolution shall always remain positive.

Both `CoordinateMapper` and `MapData` shall reject a grid resolution of zero or a negative value.

An invalid resolution shall not replace the current valid resolution.

Construction shall not produce a `CoordinateMapper` or `MapData` instance with a non-positive grid resolution.

## 3. World Boundary

### MAP-001 Default world boundary

The default world boundary shall begin at `(-1000, -1000)` and have a size of `(2000, 2000)`.

### MAP-002 Boundary containment

The world boundary shall use half-open intervals:

* `left <= x < right`
* `top <= y < bottom`

Points exactly on the left or top edge shall be inside the boundary.

Points exactly on the right or bottom edge shall be outside the boundary.

A point inside the configured world boundary shall be accepted.

A point outside the configured world boundary shall be rejected.

## 4. Obstacles

### MAP-003 Add obstacle

MapData shall support adding an obstacle using either:

* A world position
* A GridCoord

When adding an obstacle by world position:

1. Reject positions outside the world boundary.
2. Convert the accepted position to GridCoord.
3. Store one obstacle in that grid cell.

When adding an obstacle by GridCoord:

1. Convert the grid cell center to world coordinates.
2. Reject the cell when its center is outside the world boundary.
3. Store one obstacle in the accepted grid cell.

Both input forms shall follow the same world-boundary rules.

### MAP-004 Duplicate obstacle

Adding the same obstacle cell more than once shall not create duplicates.

### MAP-005 Remove obstacle

Removing an existing obstacle shall remove that grid cell.

Removing a non-existing obstacle shall leave the map unchanged.

### MAP-006 Obstacle query

Obstacle lookup by world position and lookup by GridCoord shall report the same logical cell state.

## 5. Start and Goal Poses

### MAP-007 Start pose

MapData shall support:

* Setting a start pose
* Reading the current start pose
* Clearing the current start pose

The pose shall preserve both position and heading.

### MAP-008 Goal pose

MapData shall support:

* Setting a goal pose
* Reading the current goal pose
* Clearing the current goal pose

The pose shall preserve both position and heading.

## 6. Work Zones

### MAP-009 Add work zone

A work zone with positive width and height shall be stored.

A work zone with zero or negative width or height shall not be stored.

### MAP-010 Remove work zone

A valid work-zone index shall remove the corresponding zone.

An invalid index shall return failure without modifying other zones.

## 7. Clear Map

### MAP-011 Clear editable map data

Clearing MapData shall remove:

* All obstacles
* All work zones
* Start pose
* Goal pose

Clearing MapData shall not define robot behavior, because the robot is owned outside MapData.

## 8. Save and Load

### MAP-012 Save content

The saved map shall include:

* Grid resolution
* World boundary
* Start pose position and heading, or `none`
* Goal pose position and heading, or `none`
* Obstacles
* Work zones

The saved obstacle and work-zone counts shall match the number of records written to the file.

### MAP-013 Save-load round trip

Saving a map and loading it into another MapData instance shall reproduce the same logical map state.

The reproduced state shall include:

* Grid resolution
* World boundary
* Start pose position and heading
* Goal pose position and heading
* Obstacles
* Work zones

### MAP-014 Invalid file data

Loading shall return failure without propagating an unhandled parsing exception when:

* A required numeric value is malformed.
* A required record is missing.
* A record contains an invalid number of fields.
* An unsupported record key is encountered.
* The declared obstacle count does not match the obstacle records.
* The declared work-zone count does not match the work-zone records.
* A loaded value violates this specification, including:
  * Non-positive grid resolution
  * Non-positive world-boundary size
  * Zero or negative work-zone size

Loading shall be atomic.

File data shall first be parsed and validated in temporary state.

The existing MapData state shall only be replaced after the complete file has been parsed and validated successfully.

A failed load shall leave the existing MapData state unchanged.

## 9. Out of Scope

This specification does not define:

* Grid rendering
* Window resize behavior
* Inspector scrolling
* Editor shortcuts
* Map validation rules
* A* path planning
* Robot collision behavior