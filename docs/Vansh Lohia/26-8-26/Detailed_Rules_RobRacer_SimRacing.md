# RoboRacer Sim Racing League @ IROS 2026 — Format, Rules, Scoring, Data Streams & Vehicle/Sensor Specs

Note: Rules/Timelines are subject to change.

---

## 1. Competition Format
- **Qualification:** Complete 10 consecutive autonomous laps on practice track without exceeding collision tolerance; speed doesn't matter. Required to advance.
- **Competition (Final):** Complete 10 consecutive autonomous laps as fast as possible without exceeding collision tolerance. Ranked ascending by time-to-completion.

---

## 2. Scoring, Terminology & Penalties

### 2.1 Key Terminology
- **Collision:** contact between vehicle colliders and track bounds (ground contact excluded); resets vehicle to last checkpoint.
- **Warm-Up Lap:** 1st lap; time/collisions not counted.
- **Race Laps:** next 10 laps; time/collisions counted.
- **Cool-Down Lap:** final lap; not counted, optional.
- **Checkpoints:** spaced around track, full width; finish line is the final checkpoint; locations not disclosed.

### 2.2 Penalties & Disqualification Thresholds
- **Collision penalty:** +10s per collision, cumulative (10s/20s/30s...).
- **Max collisions per race:** 10. 11+ = disqualification.
- Lap timer doesn't reset on collision; only vehicle position resets.

### 2.3 Evaluation Criteria
- **Primary metric:** total race time. Tiebreaker: best lap time / other metrics.
- Only 10 race laps scored — warm-up/cool-down excluded.
- Referee's decision is final; rebuttals only with evidence, at organizers' discretion.

### 2.4 Inspection & Code-of-Conduct Rules
- **No modifications** to: simulator executable, vehicle (chassis/powertrain/sensors), environment (track/layout/ground/weather/time), communication interface (WebSocket only), devkit (ROS 2 only), containerization (provided Docker/Dockerfile only).
- **Permitted:** any algorithm using raw perception data — reactive, map-based, raceline optimization, deep learning, RL, hybrid.
- **Not permitted:** simulation ground-truth data; controlling anything beyond vehicle actuators.
- **Serious malpractice:** tapping simulator back-end, frame-grabbing front-end, exploiting loopholes, tampering with data logging.
- **Consequences:** warning, public citation, and/or disqualification, per severity.

---

## 3. Data Streams (ROS 2 Topics)

Exposed via AutoDRIVE Devkit. **Restricted** topics: debug/training only, not usable at race run-time.

| Topic | Category | Message Type | Access |
|---|---|---|---|
| `/autodrive/roboracer_1/lidar` | Sensor | `sensor_msgs/msg/LaserScan` | Input |
| `/autodrive/roboracer_1/front_camera` | Sensor | `sensor_msgs/msg/Image` | Input |
| `/autodrive/roboracer_1/imu` | Sensor | `sensor_msgs/msg/Imu` | Input |
| `/autodrive/roboracer_1/ips` | Sensor | `geometry_msgs/msg/Point` | ⚠ Restricted |
| `/autodrive/roboracer_1/left_encoder` | Sensor | `sensor_msgs/msg/JointState` | Input |
| `/autodrive/roboracer_1/right_encoder` | Sensor | `sensor_msgs/msg/JointState` | Input |
| `/autodrive/roboracer_1/odom` | Debugging | `nav_msgs/msg/Odometry` | ⚠ Restricted |
| `/autodrive/roboracer_1/steering` | Sensor | `std_msgs/msg/Float32` | Input |
| `/autodrive/roboracer_1/steering_command` | Actuator | `std_msgs/msg/Float32` | Output |
| `/autodrive/roboracer_1/throttle` | Sensor | `std_msgs/msg/Float32` | Input |
| `/autodrive/roboracer_1/throttle_command` | Actuator | `std_msgs/msg/Float32` | Output |
| `/autodrive/reset_command` | Debugging | `std_msgs/msg/Bool` | ⚠ Restricted |
| `/autodrive/roboracer_1/collision_count` | Debugging | `std_msgs/msg/Int32` | ⚠ Restricted |
| `/autodrive/roboracer_1/lap_count` | Debugging | `std_msgs/msg/Int32` | ⚠ Restricted |
| `/autodrive/roboracer_1/lap_time` | Debugging | `std_msgs/msg/Float32` | ⚠ Restricted |
| `/autodrive/roboracer_1/best_lap_time` | Debugging | `std_msgs/msg/Float32` | ⚠ Restricted |
| `/autodrive/roboracer_1/last_lap_time` | Debugging | `std_msgs/msg/Float32` | ⚠ Restricted |
| `/tf` | Debugging | `tf2_msgs/msg/TFMessage` | ⚠ Restricted |

- `/autodrive/reset_command`: publish `True` to reset, then `False` — else it loops.

---

## 4. Vehicle Specs (Digital Twin)

**Dimensions & Mass:** Length 0.500 m · Width 0.270 m · Wheelbase 0.324 m · Track Width 0.236 m · Front/Rear Overhang 0.090 m / 0.080 m · Wheel Radius 0.059 m · Wheel Width 0.045 m · Total Mass 3.906 kg (Sprung 3.470 kg / Unsprung 0.436 kg) · Center of Mass (0.15532, 0.0, 0.01434) m · Suspension Spring 500 N/m, Damper 100 Ns/m · Tire Limits — Longitudinal: Extremum (0.15, 0.72), Asymptote (0.25, 0.464); Lateral: Extremum (0.01, 1.00), Asymptote (0.10, 0.500).

**Driving Actuator:** All-wheel drive · Throttle limits [-1, 1] normalized · Motor Torque 428 Nm · Top Speed **22.88 m/s**.

**Steering Actuator:** Ackermann steering · Limits [-1, 1] normalized → ±0.5236 rad (±30°) · Steering Rate 3.2 rad/s.

**Actuator Feedback Sensors:** Throttle & Steering sensors are virtual (instantaneous feedback loop), each outputting [-1, 1].

**Wheel Encoders:** Simulated, proprioceptive · 16 pulses/rev · ×120 conversion ratio · Outputs: Ticks, Angles.

**IPS (Indoor Positioning):** Simulated, proprioceptive · Outputs position vector [x, y, z] (m) · ⚠ Restricted (debug/training only).

**IMU:** Simulated, proprioceptive · Outputs orientation quaternion [x,y,z,w], orientation Euler angles (rad), angular velocity (rad/s), linear acceleration (m/s²).

**LIDAR:** Simulated, exteroceptive · Scan Rate 40 Hz · Angular Resolution 0.25° · 1080 measurements/scan · Range 0.06–10.0 m · Angular Range −135° to +135° (270° FOV) · Outputs: Range array, Intensity array.

**Front Camera:** Simulated, exteroceptive · FOV 48.8311° · Sensor Size 3.68×2.76 mm · Shutter 0.005 s · Focal Length 3.04 m · Aperture f/16 · Target: 16:9, 75% JPG compression · Output: RGB image.

**Frame Transforms** (relative to vehicle-local frame `roboracer_1`, rear-axle center; x=forward, y=left, z=up; translations in m, rotations in deg):

| Frame | x | y | z | R | P | Y |
|---|---|---|---|---|---|---|
| `left_encoder` / `right_encoder` | 0.0 | ±0.118 | 0.0 | 0.0 | — | 0.0 |
| `ips` / `imu` | 0.08 | 0.0 | 0.055 | 0.0 | 0.0 | 0.0 |
| `lidar` | 0.2733 | 0.0 | 0.096 | 0.0 | 0.0 | 0.0 |
| `front_camera` | −0.015 | 0.0 | 0.15 | 0.0 | 10.0 | 0.0 |
| `front_left_wheel` / `front_right_wheel` | 0.33 | ±0.118 | 0.0 | 0.0 | 0.0 | — |
| `rear_left_wheel` / `rear_right_wheel` | 0.0 | ±0.118 | 0.0 | 0.0 | — | 0.0 |

(Dashes "—" denote variable quantities, e.g., wheel spin.)

---

## 5. Environment / Track Specs ("Porto" Racetrack)

- **World frame:** `world` at [0,0,0], zero rotation — location may vary between tracks/editions.
- **Footprint:** ~30 × 10 m. **Border:** air ducts ~33 cm diameter. **Min width:** 3 car-widths (90 cm).
- **Surface:** polished concrete (flat, reflective — can challenge perception).
- **Gaps** between border ducts can let LIDAR beams pass through, appearing as open space.
- May include straights, chicanes, bifurcations, and obstacles.
- **Track changes** between phases (practice/qualification/race) and editions — participants notified in advance.
