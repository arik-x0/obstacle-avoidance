# obstacle-avoidance

A C++17 monocular obstacle avoidance system for PX4 / ArduPilot SITL drones, built on a fully micro-service architecture. Each functional boundary runs in its own thread and communicates exclusively through typed lock-free channels — no service holds a pointer to another service.

---

## Architecture overview

```
Camera HW ──► CameraService ──► DetectionService ──► DepthService ──┐
                                                                      │
MAVSDK Telemetry ──► TelemetryService ──► EKFRingBuffer ─────────────┤
                             │                                        │
                             └──► DataChannel<EKFSnapshot>           │
                                                                      ▼
                                                               SyncService
                                                                      │
                                                                      ▼
                                                       AvoidancePlannerService
                                                                      │
                             ┌────────────────────────────────────────┘
                             ▼
                         PIDService
                             │
                             ▼
                       CommandService ◄─── FlightManagerService
                             │
                             ▼
                        MAVSDK Offboard / Action
```

### Services

| Service | Thread | Input | Output |
|---|---|---|---|
| **CameraService** | T0 | Camera HW | `DataChannel<StampedFrame>` |
| **TelemetryService** | T1 | MAVSDK telemetry callbacks | `DataChannel<EKFSnapshot>` + `EKFRingBuffer` |
| **DetectionService** | T2 | `ch_frame` | `DataChannel<StampedDetection>` |
| **DepthService** | T3 | `ch_detection` | `DataChannel<DepthEstimate>` |
| **SyncService** | T5 | `ch_detection` + `ch_depth` + `EKFRingBuffer` | `DataChannel<SyncedObservation>` |
| **AvoidancePlannerService** | T4 | `ch_synced` (polls 20 Hz) | `DataChannel<VelocityCommand>` |
| **PIDService** | T6 | `ch_desired_vel` + `ch_ekf` + `ch_depth` | `CommandQueue<VelocityCommand>` |
| **CommandService** | T7 | `cq_flight` + `cq_velocity` | MAVSDK Action / Offboard |
| **FlightManagerService** | T8 | `ch_ekf` | `CommandQueue<FlightCommand>` |

### Transport primitives

- **`DataChannel<T>`** — latest-value slot. Publisher never blocks, writes overwrite the previous value. Consumers either poll `latest()` or block on `wait_next()`. Used for all sensor data where freshness matters more than completeness.
- **`CommandQueue<T>`** — bounded FIFO queue. Drops the oldest entry when the cap is reached (real-time safety). Used for flight commands and final velocity commands sent to MAVSDK.

---

## Key algorithms

### Depth estimation (DepthService)

Distance is estimated from the monocular **looming signal** — the rate at which the detected obstacle grows in the frame. A sliding window of `(coverage_ratio, timestamp)` pairs is maintained and a linear regression is fitted:

```
coverage(t) ≈ slope · t + intercept

TTC = coverage(t) / slope        (looming equation)
```

`TTC` is time-to-collision in seconds. Confidence is the R² of the fit. The 8-sector horizontal edge-density histogram from `DetectionService` is forwarded alongside the TTC for spatial threat awareness.

### EKF3 / camera synchronisation (SyncService)

The camera runs at ~30 Hz, MAVSDK telemetry at ~20 Hz, and they are asynchronous. `SyncService` resolves this by calling `EKFRingBuffer::interpolate(detection.timestamp_us)`:

1. Binary-searches the ring buffer for the two EKF snapshots bracketing the frame's capture timestamp
2. Linearly interpolates scalar fields (velocity, altitude, Euler angles)
3. **Spherically interpolates (slerp)** the attitude quaternions

The result is an `EKFSnapshot` that represents the drone's exact orientation at the moment the frame was captured.

### Body → NED rotation (RotationTracker)

The EKF3 attitude quaternion is converted to a 3×3 rotation matrix using the standard ZYX aerospace convention:

```
R_body_to_ned:

  R[0][0] = 1 - 2(qy² + qz²)     R[0][1] = 2(qxqy - qwqz)     R[0][2] = 2(qxqz + qwqy)
  R[1][0] = 2(qxqy + qwqz)        R[1][1] = 1 - 2(qx² + qz²)   R[1][2] = 2(qyqz - qwqx)
  R[2][0] = 2(qxqz - qwqy)        R[2][1] = 2(qyqz + qwqx)     R[2][2] = 1 - 2(qx² + qy²)

v_ned = R · v_body
```

Body axes: X = forward, Y = right, Z = down. This replaces the old yaw-only approximation — roll and pitch are correctly accounted for during banked manoeuvres.

### Avoidance state machine (AvoidancePlannerService)

State transitions are driven by TTC rather than raw coverage thresholds:

```
TTC > 10 s  ──► CLEAR         full cruise speed forward
TTC 6–10 s  ──► MONITORING    speed scaled down proportionally
TTC 3–6 s   ──► BRAKING       hold (zero velocity)
TTC < 2 s   ──► DODGING       lateral manoeuvre
              └► CLIMBING      if dodge is failing (CRITICAL + TTC < 1 s)
```

Dodge direction is selected from the 8-sector histogram: the drone goes towards the half of the frame with lower edge density (the clearer side). A lateral-offset PID corrects the path based on `centre_x_norm` so the drone goes around the obstacle rather than grazing it.

### PID loops (PIDService)

Three independent PID controllers refine the desired velocity before it reaches the flight controller:

| Loop | Error signal | Output |
|---|---|---|
| **Forward speed** | `TTC_target − TTC_current` | Scale factor on north/east velocity |
| **Altitude hold** | `alt_desired − alt_current` | Override of `down_m_s` |
| **Lateral offset** | Sector histogram asymmetry | Additive `east_m_s` correction |

Rate limiting (0.5 m/s per tick) prevents step commands that would cause jerky flight.

---

## Dependencies

| Library | Version | Purpose |
|---|---|---|
| [MAVSDK](https://github.com/mavlink/MAVSDK) | ≥ 2.0 | MAVLink communication (Telemetry, Action, Offboard, MavlinkPassthrough) |
| [OpenCV](https://opencv.org) | ≥ 4.0 | Camera capture, Canny edge detection, MOG2 background subtraction |
| [nlohmann/json](https://github.com/nlohmann/json) | ≥ 3.2 | JSON config parsing |
| CMake | ≥ 3.16 | Build system |
| C++17 compiler | GCC ≥ 9 / Clang ≥ 9 | Fold expressions, structured bindings |

---

## Building

```bash
# Release build
./scripts/build.sh

# Debug build (AddressSanitizer + UBSan)
./scripts/build.sh --debug

# Custom MAVSDK install path
./scripts/build.sh --prefix /opt/mavsdk

# Clean rebuild
./scripts/build.sh --clean

# Manual CMake
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . --parallel
```

> **IDE / clangd support**: CMakeLists.txt has `CMAKE_EXPORT_COMPILE_COMMANDS=ON`. After the first configure, copy `build/compile_commands.json` to the project root so clangd resolves all headers:
> ```bash
> cp build/compile_commands.json .
> ```

---

## Running with ArduPilot SITL

**1. Start SITL + Gazebo**

```bash
# Default (with Gazebo GUI and MAVProxy console)
./scripts/launch_sitl.sh

# Headless (no Gazebo window, useful on servers)
./scripts/launch_sitl.sh --headless

# No GCS (no MAVProxy, saves resources)
./scripts/launch_sitl.sh --no-gcs
```

Set `ARDUPILOT_HOME` if your ArduPilot checkout is not at `~/ardupilot`:

```bash
ARDUPILOT_HOME=/path/to/ardupilot ./scripts/launch_sitl.sh
```

**2. Run the avoidance system**

```bash
./build/obstacle_avoidance config/params.json
```

The system connects to `udpin://0.0.0.0:14540` by default. It will wait up to 30 s for the autopilot to appear, then sit in `CONNECTED` phase until armed.

**3. Arm and take off**

Either set `auto_arm_takeoff: true` in `config/params.json`, or send an ARM command from QGroundControl / MAVProxy. Once armed the system takes off automatically to `takeoff_alt_m`, then enters offboard mode and starts avoidance.

**4. Shut down gracefully**

Press `Ctrl-C`. The SIGINT handler triggers a land request — the system stops offboard, issues a LAND command, waits for touchdown, disarms, and exits cleanly.

---

## Configuration

All parameters live in `config/params.json`. Key sections:

```json
{
  "mavsdk": {
    "connection_url": "udpin://0.0.0.0:14540",
    "connect_timeout_s": 30.0,
    "telemetry_rate_hz": 20.0,
    "enable_debug_sniffer": true
  },
  "depth": {
    "window_size": 10,
    "ttc_max_s": 30.0
  },
  "avoidance": {
    "ttc_clear_s": 10.0,
    "ttc_warn_s": 6.0,
    "ttc_brake_s": 3.0,
    "ttc_dodge_s": 2.0,
    "cruise_speed_m_s": 3.0,
    "dodge_speed_m_s": 2.0
  },
  "pid": {
    "ttc_target_s": 5.0,
    "fwd_kp": 0.15,
    "alt_kp": 0.8,
    "lat_kp": 1.2
  },
  "control": {
    "takeoff_alt_m": 5.0,
    "auto_arm_takeoff": false
  }
}
```

---

## Source layout

```
src/
  core/
    messages.hpp             All inter-service message types
    channel.hpp              DataChannel<T>  – latest-value slot
    command_queue.hpp        CommandQueue<T> – bounded FIFO
    service_base.hpp         IService interface + thread lifecycle
    ekf_ring_buffer.hpp      EKF history with slerp interpolation
    mavsdk_connection.hpp/.cpp  MAVSDK system discovery
  navigation/
    rotation_tracker.hpp/.cpp   Quaternion → R matrix, body↔NED transforms
  services/
    camera_service.*
    detection_service.*
    depth_service.*
    telemetry_service.*
    sync_service.*
    avoidance_planner_service.*
    pid_service.*
    command_service.*
    flight_manager_service.*
  camera/
    camera_interface.*       OpenCV VideoCapture wrapper (used by CameraService)
  detection/
    object_detector.*        Canny edge / MOG2 detector (used by DetectionService)
  mavsdk_manager/
    mavsdk_debug.hpp         MAVLink message sniffer, ACK tracker, heartbeat watchdog
  utils/
    logger.hpp               Thread-safe coloured logger
  main.cpp                   Channel construction, service wiring, signal handling
config/
  params.json                All tunable parameters
scripts/
  build.sh                   CMake build helper
  launch_sitl.sh             ArduPilot SITL + Gazebo launcher
```

---

## MAVLink debug output

`TelemetryService` owns a `MavsdkDebugger` that hooks into `MavlinkPassthrough` and produces a statistics table every 5 seconds:

```
╔══════════════════════════════════════════════════════════════╗
║           MAVSDK Debug Statistics  (+12.3s)
╠══════════════╦═══════╦══════════╦══════════╦══════════╦══════╣
║ Message      ║ Count ║  Rate Hz ║  min gap ║  max gap ║ avg gap ║
╠══════════════╬═══════╬══════════╬══════════╬══════════╬══════╣
║ HEARTBEAT    ║   246 ║      1.0 ║  998ms   ║ 1003ms   ║ 1000ms ║
║ ATTITUDE     ║  4920 ║     20.0 ║   49ms   ║   51ms   ║   50ms ║
║ ...
```

Command ACK round-trip times are tracked for every ARM / TAKEOFF / LAND command. STATUSTEXT messages from the flight controller are mirrored to the logger in real time.

---

## License

See [LICENSE.md](LICENSE.md).
