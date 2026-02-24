# Lab 4: Follow the Gap

This package implements Lab 4 reactive obstacle avoidance in ROS 2.

Primary method:
- **Follow-the-Gap** (default, assignment pipeline)

Optional method:
- **Obstruction geometry** (alternate strategy kept for experimentation/comparison)

---

## How we implemented each algorithm step

All logic is in `gapfinder/__init__.py`.

### Step 1: Obtain and preprocess LiDAR scans
Lab requirement: clean scan data before gap detection.

What we implemented:
- Convert ranges to a NumPy array.
- Clip values to `[range_min, range_max]`.
- Replace `NaN` and `inf` with `max_range`.
- Optionally smooth ranges with a moving average window.

Code:
- `preprocess_lidar(...)`
- `smooth_lidar(...)`

---

### Step 2: Find the closest obstacle
Lab requirement: identify the nearest LiDAR point.

What we implemented:
- Use `np.argmin(ranges)` to get the closest index.
- Read the closest distance at that index.

Code:
- `create_safety_bubble(...)`

---

### Step 3: Create a safety bubble (mask unsafe directions)
Lab requirement: set points inside a bubble around the closest obstacle to `0`.

What we implemented:
- Compute bubble angular size with `atan(bubble_radius / closest_dist)`.
- Convert angular size to number of LiDAR points using `angle_increment`.
- Zero all points in the bubble index range.

Code:
- `create_safety_bubble(...)`

---

### Step 4: Find the maximum gap
Lab requirement: find the longest sequence of non-zero points.

What we implemented:
- Scan the array for contiguous non-zero runs.
- Track and return the longest run as `(start_idx, end_idx)`.

Code:
- `find_max_gap(...)`

---

### Step 5: Find the best point in the max gap
Lab requirement: select a goal point in that gap.

What we implemented:
- Naive method: choose the farthest point (`argmax`) inside `[start_idx, end_idx]`.
- Convert that index into steering angle in radians.

Code:
- `find_best_point(...)`
- `get_drive_vector_follow_the_gap(...)`

---

### Step 6: Actuate the car
Lab requirement: publish `AckermannDriveStamped` to `/drive`.

What we implemented:
- Compute steering angle from selected goal point.
- Compute speed using:
  - time-to-impact cap,
  - top-speed cap,
  - turn-rate cap (`max_accel`-based).
- Publish drive command to `/drive`.

Code:
- `callback(...)`

---

## Runtime strategy switch

- `use_follow_the_gap = true` → Follow-the-Gap (assignment method)
- `use_follow_the_gap = false` → Obstruction geometry

For obstruction geometry, steering is explicitly converted to radians before publishing.

---

## Run

```bash
ros2 run gapfinder gap_follow
```

Or:

```bash
ros2 run gapfinder gapfinder
```

Both commands run the same node entrypoint.

---

## Parameters

- `use_follow_the_gap` (bool): strategy selector (`true` by default).
- `bubble_radius` (float): safety-bubble radius in meters (default `0.4`).
- `smooth_window` (int): smoothing window size (set `0` to disable smoothing; default `5`).
- `flip_scan` (bool): reverse LiDAR ordering if your scanner publishes right-to-left (default `false`).
- `use_front_window` (bool): limit Follow-the-Gap to a front angular window (default `false`).
- `front_window_degrees` (float): half-window angle in degrees when `use_front_window=true` (default `90.0`, i.e., +/-90 degrees).

---

## Figures (alternative obstruction model)

![](resource/drive-strategy.png)

![](./resource/obstruction-map.png)
