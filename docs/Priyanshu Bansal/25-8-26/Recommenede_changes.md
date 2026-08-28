# Architecture and Workflow Analysis — Autodrive RoboRacer Stack

This document provides a complete overview of the architecture, workflow, identified structural faults, and recommended code changes for the [Autodrive_proboticists](https://github.com/srinjoy-not-007/Autodrive_proboticists) repository.

---

## 1. Complete Architecture Overview

The system is designed as a ROS 2 package (`pit_crew`) built on top of the official competition environment (ROS 2 Humble / Ubuntu 22.04), interacting with a Unity-based simulator (RoboRacer). 

### Data Flow & Component Diagram

```mermaid
graph TD
    %% Offline Pipeline
    subgraph Offline Pipeline (Track Setup)
        A[1. Reconnaissance Lap] -->|FTG reactive driving| B(ROS 2 Bag: LiDAR/IMU/Encoders)
        B -->|build_map_from_bag.py| C{Map Creation}
        C -->|Uncorrected Offset 0.0| D[data/maps/track]
        C -->|Corrected Offset 0.1933| E[data/maps/track_c]
        D -->|make_track_lines.py| F[Select Best Map & Path]
        E -->|make_track_lines.py| F
        F -->|Promoted Canonical Files| G[data/racelines/track_*]
        G -->|footprint_analysis.py| H[Offline Footprint Sweep Validation]
    end

    %% Online Stack
    subgraph Online Runtime Stack (Racing)
        I[/autodrive/roboracer_1/lidar] --> J(localization.py - Monte Carlo PF)
        K[/autodrive/roboracer_1/imu] --> J
        L[/autodrive/roboracer_1/encoders] --> J
        J -->|/pit_crew/pose| M(mpc.py - Kinematic Bicycle MPC)
        G -.->|Loads chosen csv line| M
        M -->|/autodrive/roboracer_1/throttle_command| N[Simulator Actuation]
        M -->|/autodrive/roboracer_1/steering_command| N
    end
```

---

## 2. Core Workflow Walkthrough

When a new (previously unseen) track is released (e.g., during the 3-day finals window), the team runs the script `scripts/new_track_drill.sh`. The workflow is split into five distinct stages:

1. **Reconnaissance**: The car drives blind using the reactive `follow_the_gap` controller (mapless obstacle avoidance). Raw sensor topics (LiDAR, IMU, wheel encoders, IPS, commands) are recorded into a ROS 2 bag.
2. **Mapping**: `build_map_from_bag.py` generates occupancy grid maps (`.pgm` and `.yaml`). It creates two variants:
   - **Uncorrected** (`LIDAR_OFFSET = 0.0`)
   - **Corrected** (`LIDAR_OFFSET = 0.1933` m) which compensates for the physical placement of the LiDAR sensor relative to the vehicle's center of pose.
3. **Racing Line Generation**: `make_track_lines.py` runs `extract_centerline.py` and `optimize_raceline.py` on both maps to generate:
   - `safe.csv`: A centerline-following path (safer, wider margins).
   - `raceline.csv`: A minimum-curvature optimized line.
4. **Evaluation & Selection**: It runs clearance, steering-lock, and velocity checks on all generated paths. It selects the map and line combination that yields the fastest drivable time while clearing the minimum safety clearance threshold ($\ge 0.22$ m), promoting them to canonical files (e.g. `track_race.csv`).
5. **Footprint Check**: `footprint_analysis.py` does a swept-volume collision check on the selected line before it is driven.
6. **Racing**: The stack is launched using `submission/autodrive_devkit.sh`. The `localization` node runs a particle filter against the promoted map, and the `mpc` node tracks the generated race line.

---

## 3. Structural Faults & Recommended Changes

Four major structural faults and platform limitations were identified in the codebase. Below is a detailed description of each, along with copy-paste-ready code fixes.

### Fault 1: Offline Particle Filter Discrepancies (`offline_pf.py` & `pf_core.py`)
- **Symptoms**: The offline tuning script `offline_pf.py` diverges constantly (e.g. 192 relocalizations) and does not reproduce live driving behavior.
- **Root Causes**:
  1. **Missing LiDAR Offset**: `pf_core.py` calculates scan endpoints assuming the LiDAR is at the pose center. In contrast, the live `localization.py` uses the `lidar_offset` (0.1933 m).
  2. **Jitter/Time-Sync Drift**: `offline_pf.py` uses the host database recording timestamp `t` to integrate prediction steps (`dt` for `LaserScan` and `Imu`), but uses the simulation message header timestamp (`m.header.stamp`) for encoder speeds. If the simulator lags (running at $< 1.0\times$ speed), these two time bases drift apart, causing large prediction errors.
  3. **Brittle Wheel Name Parsing**: The wheel key is extracted using `name[-13]`. This brittle substring slice evaluates `/autodrive/roboracer_1/left_encoder` to `/` and `/autodrive/roboracer_1/right_encoder` to `r`, which does not match the `'L'` and `'R'` keys used in `localization.py`.

#### Recommended Fixes:
Update **`src/pit_crew/pf_core.py`** to support the `lidar_offset`:
```diff
--- src/pit_crew/pf_core.py
+++ src/pit_crew/pf_core.py
@@ -48,4 +48,6 @@
         self.ox, self.oy = meta['origin'][0], meta['origin'][1]
         self.H, self.W = dist.shape
         self.centerline = centerline
+        # Load lidar_offset from meta yaml if present, otherwise fall back to params
+        self.lidar_offset = float(meta.get('lidar_offset', params.get('lidar_offset', 0.0)))
 
         n = self.p['n_particles']
@@ -118,4 +120,6 @@
         head = (imu_yaw + self.pth)[:, None]
         world_ang = head + ang[None, :]
-        ex = self.px[:, None] + rr[None, :] * np.cos(world_ang)
-        ey = self.py[:, None] + rr[None, :] * np.sin(world_ang)
+        sx = self.px[:, None] + self.lidar_offset * np.cos(head)
+        sy = self.py[:, None] + self.lidar_offset * np.sin(head)
+        ex = sx + rr[None, :] * np.cos(world_ang)
+        ey = sy + rr[None, :] * np.sin(world_ang)
```

Update **`src/tools/offline_pf.py`** to use synchronized message header timestamps and robust wheel parsing:
```diff
--- src/tools/offline_pf.py
+++ src/tools/offline_pf.py
@@ -107,8 +107,9 @@
             if args.speed_source == 'imu':
+                stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
                 if imu_last_t is not None:
-                    d = min(max(t - imu_last_t, 0.0), 0.2)
+                    d = min(max(stamp - imu_last_t, 0.0), 0.2)
                     imu_speed += m.linear_acceleration.x * d
                     imu_speed *= 0.995          # leak against integrator drift
                     imu_speed = float(np.clip(imu_speed, -1.0, 20.0))
-                imu_last_t = t
+                imu_last_t = stamp
         elif name.endswith('ips'):
@@ -120,3 +121,4 @@
             if m.position:
-                speed_est.update(name[-13], stamp, m.position[0])
+                wheel = 'L' if 'left' in name else 'R'
+                speed_est.update(wheel, stamp, m.position[0])
         elif name.endswith('lidar'):
@@ -124,6 +126,7 @@
                 continue
+            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
             if not pf.initialized:
                 pf.init_near(truth[0], truth[1])
-                last_t = t
+                last_t = stamp
                 continue
-            dt = min(max(t - last_t, 0.0), 0.5)
-            last_t = t
+            dt = min(max(stamp - last_t, 0.0), 0.5)
+            last_t = stamp
```

---

### Fault 2: Map-Selection & LiDAR Offset Parameter Loss
- **Symptoms**: Correcting the LiDAR offset in map generation yields a 1.6s lap time improvement on Porto. However, on ICRA, it results in centerline extraction failures. Thus, `new_track_drill.sh` generates both uncorrected and corrected maps, but when it promotes the winner to the canonical name (overwriting `${TRACK}.yaml` and `${TRACK}.pgm`), the runtime `localization.py` node does not know whether the canonical map was corrected or uncorrected, running with the default `lidar_offset: 0.0`.
- **Root Cause**: The sensor origin used during mapping is lost on file promotion.
- **Recommended Fix**: Save the `lidar_offset` directly into the map `.yaml` metadata file during construction, and have the localization node read it.

Update **`src/tools/build_map_from_bag.py`** to save the offset:
```diff
--- src/tools/build_map_from_bag.py
+++ src/tools/build_map_from_bag.py
@@ -185,3 +185,4 @@
                 f"resolution: {RES}\n"
                 f"origin: [{xmin}, {ymin}, 0.0]\n"
-                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
+                f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n"
+                f"lidar_offset: {LIDAR_FROM_POSE}\n")
```

Update **`src/pit_crew/localization.py`** to load the offset:
```diff
--- src/pit_crew/localization.py
+++ src/pit_crew/localization.py
@@ -107,4 +107,8 @@
         self.res = float(meta['resolution'])
         self.ox, self.oy = meta['origin'][0], meta['origin'][1]
+        # Load lidar_offset from map yaml if present, overriding default parameter
+        if 'lidar_offset' in meta:
+            self.lidar_offset = float(meta['lidar_offset'])
+            self.get_logger().info(f'Loaded lidar_offset {self.lidar_offset} from map yaml')
         occupied = img <= 50
```

---

### Fault 3: Platform Portability Bug (`cp` command crash on Windows)
- **Symptoms**: Running `tools/make_track_lines.py` on Windows crashes with a `FileNotFoundError` because the script attempts to call `cp` via `subprocess.run()`.
- **Root Cause**: Windows command line lacks the `cp` executable.
- **Recommended Fix**: Use Python's built-in `shutil.copy` which is platform-agnostic.

Update **`tools/make_track_lines.py`**:
```diff
--- tools/make_track_lines.py
+++ tools/make_track_lines.py
@@ -34,4 +34,5 @@
 import subprocess
 import sys
+import shutil
 
 sys.path.insert(0, 'tools')
@@ -186,7 +187,7 @@
     for src, dst in ((chosen['centerline'] + '.csv', f'{track}_centerline'),
                      (chosen['safe']['path'] + '.csv', f'{track}_safe')):
-        subprocess.run(['cp', src, f'data/racelines/{dst}.csv'], check=True)
+        shutil.copy(src, f'data/racelines/{dst}.csv')
     if 'raceline' in chosen:
-        subprocess.run(['cp', chosen['raceline']['path'] + '.csv',
-                        f'data/racelines/{track}_raceline.csv'], check=True)
+        shutil.copy(chosen['raceline']['path'] + '.csv',
+                    f'data/racelines/{track}_raceline.csv')
 
@@ -198,4 +199,4 @@
     kind, win = min(viable, key=lambda kv: kv[1]['lap'])
-    subprocess.run(['cp', win['path'] + '.csv',
-                    f'data/racelines/{track}_race.csv'], check=True)
+    shutil.copy(win['path'] + '.csv', f'data/racelines/{track}_race.csv')
```

---

### Fault 4: Teleport jumps / Reset artifacts in recon trajectory
- **Symptoms**: The reconnaissance lap involves collisions which teleport the car back to the last checkpoint. These jumps appear in the recorded trajectory `_traj.csv`. When `extract_centerline.py` interpolates the path, the centerline wiggles or cuts through walls and obstacles.
- **Root Cause**: The coordinates jump abruptly, but the interpolation assumes a continuous, non-jumping path.
- **Recommended Fix**: Detect teleport jumps in `extract_centerline.py`, trace back to the closest past point (where the car was reset to), and excise the failed "retry" segment of the trajectory, stitching a clean continuous loop.

Update **`tools/extract_centerline.py`**:
```diff
--- tools/extract_centerline.py
+++ tools/extract_centerline.py
@@ -40,4 +40,28 @@
 
+def clean_teleports(tx, ty):
+    tx = list(tx)
+    ty = list(ty)
+    i = 1
+    while i < len(tx):
+        dx = tx[i] - tx[i-1]
+        dy = ty[i] - ty[i-1]
+        if math.hypot(dx, dy) > 0.5:
+            # Teleport detected! Find the closest point in the past
+            checkpoint = (tx[i], ty[i])
+            best_j = -1
+            min_d = 1.5 # max distance to match a past checkpoint
+            # Search in the past, but not too close to the current point
+            for j in range(max(0, i - 300), i - 15):
+                d = math.hypot(tx[j] - checkpoint[0], ty[j] - checkpoint[1])
+                if d < min_d:
+                    min_d = d
+                    best_j = j
+            if best_j != -1:
+                # Excise the failed segment
+                del tx[best_j:i]
+                del ty[best_j:i]
+                # Resume checking from the excised join point
+                i = max(1, best_j)
+                continue
+        i += 1
+    return np.array(tx), np.array(ty)
+
+
 def prune_to_cycle(skel):
@@ -238,4 +262,5 @@
     free, occ, res, ox, oy = load_map(prefix)
     tx, ty = load_traj(prefix + '_traj.csv')
+    tx, ty = clean_teleports(tx, ty)
```
