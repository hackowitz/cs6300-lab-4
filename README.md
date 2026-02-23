# Lab 4: Follow the Gap

This package implements reactive obstacle avoidance with two selectable strategies:

- **Follow-the-Gap** (default)
- **Obstruction geometry** (original method)

## Algorithm (per lab spec)

1. **Preprocess** laser scans: clip to valid range, replace NaN/inf with max range, optional smoothing.
2. **Closest point** — find the nearest obstacle in the LiDAR ranges.
3. **Safety bubble** — set all points inside a bubble around that closest point to 0. Everything else is free space (gaps).
4. **Find max gap** — longest run of consecutive non-zero elements.
5. **Best point** — choose a goal in that gap (e.g. farthest point).
6. **Actuate** — publish `AckermannDriveStamped` to `/drive` to steer toward the goal.

## Run

```bash
ros2 run gapfinder gap_follow
```

Or: `ros2 run gapfinder gapfinder` (same node).

## Parameters

- `use_follow_the_gap` (bool): `true` uses Follow-the-Gap, `false` uses obstruction geometry.
- `bubble_radius` (float): radius of the safety bubble in meters (default 0.4).
- `smooth_window` (int): smoothing window size; set to 0 to disable (default 5).

## Figures (from original write-up)

The repo also includes visualizations of an alternative obstruction model (arc geometry):

![](resource/drive-strategy.png)

![](./resource/obstruction-map.png)
