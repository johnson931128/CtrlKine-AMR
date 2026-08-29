# Phase: Correlative Scan Matching

## Algorithm

Use odometry only to produce the predicted pose. Select a bounded cyclically
uniform subset of physical-hit beams, transform endpoints with full sensor
extrinsics, and score them against SLAM-owned occupied evidence.

Search configured x/y/yaw offsets at coarse resolution, then refine around the
best coarse candidate. Unknown and out-of-bounds endpoints score zero. Require a
minimum number of usable/in-bounds endpoints and a configured normalized score.

Tie breaking is deterministic: higher score, then smaller normalized correction,
then smaller absolute yaw, then fixed lexical offset order. Matching is
read-only and must not change the grid revision.

## Diagnostics

`ScanMatchResult` records predicted/corrected poses, attempted/accepted state,
reason, best score, selected/used beams, coarse/fine candidate counts, and the
x/y/yaw correction.

## Gate

Tests cover x/y/yaw recovery, combined correction, ties, low support,
all-max/invalid scans, bounds, extrinsics, full-circle selection, deterministic
candidate counts, and no map mutation. Benchmark complete matching separately
from map integration.
