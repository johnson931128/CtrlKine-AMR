# Phase: SLAM Frontend

## State flow

```text
OdometryDelta -> prediction -> scan validation -> scan matching
              -> corrected pose -> guarded scan integration -> updated map
```

States are `Uninitialized`, `Tracking`, and `Lost`.

- A valid informative first scan bootstraps at local identity and enters
  `Tracking` without matching the empty map.
- An invalid or all-max first scan leaves the frontend `Uninitialized`.
- Tracking applies valid odometry before matching.
- An accepted match integrates at the corrected pose and clears failures.
- Invalid/all-max scans use odometry prediction only, do not claim correction,
  and do not integrate.
- Poor matches do not integrate and increment consecutive failures.
- Excessive odometry motion enters `Lost` immediately.
- Lost keeps its map frozen and may attempt bounded reacquisition. Only an
  accepted informative match returns to `Tracking`.
- Reset deterministically clears pose, map, lifecycle, and diagnostics.

## Gate

Tests prove integration ordering, map revision behavior, bootstrap, invalid and
all-max policies, predicted pose math, rejection, Lost entry, reacquisition, and
reset independence.
