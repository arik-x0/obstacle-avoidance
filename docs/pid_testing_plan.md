# PID + APF + CBF Testing Plan

A structured guide for validating the control stack in SITL before any real hardware flight.
Each section lists: what to test, how to observe it, expected behaviour, and known doubts.

---

## 1. Altitude Hold PID

### What it does
Corrects `down_m_s` every tick to keep the drone at `target_alt_m_` (set by FlightManager on takeoff).
Uses derivative-on-measurement and a first-order D-term filter (τ = 0.1 s).

### Tests

| # | Scenario | How to run | Expected |
|---|----------|-----------|----------|
| 1a | Hover at 5 m, no disturbance | Arm + takeoff, do nothing for 60 s | `alt_rel_m` stays within ±0.3 m of 5 m |
| 1b | Step disturbance upward | Command a brief climb via GCS, then release | Returns to 5 m within 3–5 s without oscillation |
| 1c | Step disturbance downward | Same, push down | Returns to 5 m; does not overshoot by >0.5 m |
| 1d | Change target altitude mid-flight | Call `set_target_altitude(8.0f)` | Climbs smoothly; no derivative kick (D-on-measurement) |
| 1e | Simultaneous horizontal + altitude correction | Fly forward while altitude PID corrects | `down_m_s` correction does not spill into horizontal velocity |

### Log lines to watch
```
[PIDService] Started
```
Add temporary debug print inside the altitude block:
```cpp
logger_.debug("[ALT] err=", alt_err, " corr=", raw_correction, " out=", output.down_m_s);
```

### Doubts / risks
- **Gains are guesses** (`alt_kp=0.8`, `alt_ki=0.05`, `alt_kd=0.2`). Real EKF altitude noise may make the D-term fight the filter — symptom: buzzing/oscillation in `down_m_s`.
- **Barometer lag** in SITL: EKF altitude can lag 100–300 ms behind truth. The D-filter (τ=0.1 s) may not be enough.
- **Anti-windup edge case**: if the drone is physically limited from climbing (e.g., low throttle authority) the integrator may still run. Watch `alt_integral_` in logs.

### Tuning guide
| Symptom | Action |
|---------|--------|
| Oscillation (altitude bouncing) | Reduce `alt_kd`, or increase `alt_deriv_tau` |
| Slow return to target | Increase `alt_kp` or `alt_ki` |
| Overshoot on recovery | Reduce `alt_kp`, check `alt_max_integral` |
| Buzzing output | Increase `alt_deriv_tau` (smoother D-term) |

---

## 2. Forward Speed PID (TTC-based)

### What it does
Scales horizontal velocity by a factor ∈ [0, 1.5] to maintain `ttc_target_s = 5 s` of time-to-collision.
Only activates when `depth.confidence > 0.2` AND `TTC < ttc_target_s × 1.5`.

### Tests

| # | Scenario | Expected |
|---|----------|----------|
| 2a | Open sky, no obstacle | PID inactive; scale = 1.0; full cruise speed |
| 2b | Approaching obstacle slowly | Scale gradually decreases; drone slows before reaching target TTC |
| 2c | Obstacle very close (TTC < 5 s) | Scale < 1 (drone slows or stops) |
| 2d | Obstacle disappears mid-flight | Integral resets to 0; no burst of speed when next obstacle appears |
| 2e | Low-confidence detection (`conf < 0.2`) | PID inactive; scale = 1.0 regardless of TTC |

### Log lines to watch
Add inside the forward PID block:
```cpp
logger_.debug("[FWD] TTC=", depth.ttc_s, " scale=", scale, " integral=", fwd_integral_);
```

### Doubts / risks
- **TTC is speed-dependent**: TTC = distance / approach_speed. At slow speeds TTC can be artificially large even for a near obstacle. The PID may think it has plenty of time while the drone is only 1 m away.
- **Confidence threshold (0.2)** is arbitrary. In dark environments or at low contrast the detector may give `conf = 0.19` and the PID stays off even with a real obstacle at 2 m.
- **Scale can exceed 1.0 (max 1.5)**: When TTC is large and fwd_err is positive the drone can speed up beyond the desired velocity. Verify this is intentional in your use case.
- **Integral reset timing**: On obstacle loss the integral resets immediately. If the obstacle reappears within the same tick the PID starts from zero — acceptable but could cause a brief speed burst.

### Tuning guide
| Symptom | Action |
|---------|--------|
| Drone barely slows for obstacle | Increase `fwd_kp` |
| Drone overshoots stop, then bounces | Reduce `fwd_kp`, increase `fwd_kd` |
| Slow response to changing TTC | Reduce `fwd_deriv_tau` |
| Noisy D-term (scale jitters) | Increase `fwd_deriv_tau` |

---

## 3. Artificial Potential Fields (APF) Guidance

### What it does
Combines an attractive force (forward cruise), a repulsive force (away from obstacle weighted by TTC and confidence), and velocity damping. Rotates result from body frame to NED before publishing.

### Tests

| # | Scenario | Expected |
|---|----------|----------|
| 3a | Open field, no obstacle | `F_rep=0`; drone flies straight forward at ~`k_att = 3 m/s` |
| 3b | Obstacle dead ahead, centred | Repulsive force pushes backward; drone decelerates or stops |
| 3c | Obstacle slightly left of centre | Repulsive force has rightward component; drone steers right |
| 3d | Obstacle disappears | `state_` → CLEAR; repulsive force drops to zero |
| 3e | Emergency: TTC < 1.5 s | `state_` → CLIMBING; `down_m_s = -ascent_speed_m_s` |
| 3f | Stuck scenario (slow speed + obstacle present for >20 ticks) | Lateral kick injected; alternates left/right every 20 ticks |

### Log lines to watch
```
[Planner] → ACTIVE  TTC=X s  conf=Y  |F|=Z
[Planner] → CLIMBING – TTC=X s
[Planner] Stuck escape kick LEFT  ticks=21
```

Add per-tick force debug:
```cpp
logger_.debug("[APF] F_att=(", f_att_x, ",", f_att_y,
              ") F_rep=(", f_rep_x, ",", f_rep_y,
              ") F_damp=(", f_damp_x, ",", f_damp_y,
              ") angle_deg=", obstacle_angle * 180.f / M_PI, " TTC=", ttc);
```

### Doubts / risks
- **No global goal**: The attractive force is always forward in body frame. The drone never checks for a waypoint or mission. It will fly forever unless commanded otherwise.
- **Local minima**: A symmetric obstacle directly ahead can produce zero net lateral force. The stuck-detection kicks help but the kick direction alternates blindly — it may kick toward a wall.
- **Repulsion formula is TTC-space Khatib**: This assumes TTC linearly relates to distance. At very low approach speed the formula underestimates real proximity.
- **Sector histogram centroid**: The obstacle angle is a weighted average. If two separate obstacles appear on left and right, the centroid points forward — repulsion goes backward, not sideways. The drone would slow but not steer.
- **k_att = 3.0 m/s** sets cruise speed. After damping and max_speed clamping the actual forward speed in SITL may differ. Verify against EKF `vel_north`.

---

## 4. Control Barrier Function (CBF) Safety Filter

### What it does
After all PID corrections and rate limiting, projects the NED velocity onto the safe halfspace defined by `h = TTC - cbf_ttc_min_s ≥ 0`. Only activates when `h < cbf_activate_h = 3.0` and `confidence > 0.2`.

### Tests

| # | Scenario | Expected |
|---|----------|----------|
| 4a | `h > 3.0` (far from obstacle) | CBF returns command unchanged |
| 4b | `h ∈ [0, 3.0]` with safe velocity | `cbf_val ≥ 0`; command unchanged |
| 4c | `h ∈ [0, 3.0]` with unsafe velocity | `cbf_val < 0`; velocity modified; debug log printed |
| 4d | `h < 0` (TTC below hard minimum) | CBF still fires; large correction applied |
| 4e | `cbf_enabled = false` in params.json | CBF block skipped entirely; command passed through raw |

### Log lines to watch
```
[PIDService] CBF: h=X val=Y λ=Z
```
This only prints when the constraint is violated. If you never see it in a test with an obstacle, either:
- The obstacle never got close enough (`h > cbf_activate_h`), or
- The upstream PID already slowed the drone enough that the CBF constraint was already satisfied.

### Doubts / risks
- **∇h is approximate**: The true gradient of TTC w.r.t. velocity requires knowing the exact geometry. Here `∇h = [−cos(angle), −sin(angle)]` assumes TTC decreases linearly with forward speed toward the obstacle centroid. This is only exact for a single point obstacle directly ahead.
- **CBF constraint is 2D only**: The filter only modifies `north_m_s` and `east_m_s`. Vertical approach to an obstacle (e.g., flying into a ceiling) is not protected.
- **γ = 1.0**: Higher γ gives a tighter, faster-responding barrier but can cause chattering near the boundary. Lower γ is smoother but allows the drone to get closer before correcting.
- **cbf_ttc_min_s = 1.5 s**: At 3 m/s cruise, 1.5 s ≈ 4.5 m. This is the hard floor. If latency in the pipeline (camera → detection → depth → PID) is >500 ms, the drone may breach 1.5 s TTC before the CBF sees it.
- **Does not account for rate limiting**: Rate limiting runs before CBF. A large CBF correction might be itself rate-limited on the next tick — reducing the safety guarantee under high gain conditions.

---

## 5. Rate Limiting

### What it does
Clamps the change in `north_m_s`, `east_m_s`, `down_m_s` per tick to `max_delta_speed_per_tick = 0.5 m/s`.
At 20 Hz loop rate this allows at most 10 m/s² acceleration command — realistic for multirotor.

### Tests

| # | Scenario | Expected |
|---|----------|----------|
| 5a | Step from 0 → 5 m/s desired | Output ramps at 0.5 m/s per tick (25 ticks = 1.25 s to full speed) |
| 5b | Emergency stop (desired → 0) | Output ramps down at same rate; not instant |
| 5c | Emergency climb (APF → CLIMBING) | `down_m_s` ramps to −`ascent_speed_m_s` over multiple ticks |

### Doubts / risks
- **Rate limiting before CBF**: If the CBF needs a large sudden correction it may be slowed by the limiter from the previous tick. For very fast obstacles this is a concern.
- **`prev_output_` initialised to zero**: On first tick the rate limiter starts from zero. If the drone is already moving when the system starts, the first command may be wrong.

---

## 6. End-to-End Integration Tests

Run these after each individual subsystem checks out.

| # | Scenario | Pass condition |
|---|----------|---------------|
| E1 | Takeoff to 5 m, hover 60 s, land | Altitude ±0.3 m throughout; no oscillation |
| E2 | Fly forward 50 m, no obstacles | Straight line at cruise speed; altitude stable |
| E3 | Fly toward a static wall, stop at ~4.5 m (1.5 s TTC at 3 m/s) | CBF log appears; drone stops before impact |
| E4 | Fly toward wall, obstacle disappears (detector confidence drops) | Drone resumes cruise; no jerk |
| E5 | Two obstacles: one left, one right | Drone slows (sector centroid ≈ forward); may need manual intervention |
| E6 | Narrow corridor (walls on both sides) | Known limitation — likely gets stuck; stuck-kick should engage |

---

## 7. Parameters Quick Reference

| Parameter | Current value | Effect of increasing |
|-----------|--------------|---------------------|
| `alt_kp` | 0.8 | Faster altitude response → risk of overshoot |
| `alt_ki` | 0.05 | Eliminates steady-state error → risk of windup |
| `alt_kd` | 0.2 | Damping → risk of noise amplification |
| `alt_deriv_tau` | 0.1 s | Smoother D-term → slower D response |
| `fwd_kp` | 0.15 | Stronger speed scaling → tighter TTC tracking |
| `fwd_ki` | 0.02 | Eliminates TTC steady-state error |
| `fwd_kd` | 0.05 | D-term on TTC error |
| `fwd_deriv_tau` | 0.15 s | Smoother D-term |
| `k_att` | 3.0 | Higher cruise speed |
| `k_rep` | 20.0 | Stronger repulsion → more aggressive steering |
| `k_damp` | 0.3 | More damping → less oscillation at equilibrium |
| `cbf_gamma` | 1.0 | Tighter CBF → earlier intervention |
| `cbf_ttc_min_s` | 1.5 s | Larger hard-minimum safety margin |
| `cbf_activate_h` | 3.0 | CBF activates earlier (further from obstacle) |

---

## 8. Known Limitations (not bugs, design choices)

1. **No global planner**: The system reacts locally. It cannot plan a path around an obstacle and resume a mission.
2. **TTC only, no absolute distance**: If the drone is stationary and an obstacle moves toward it, TTC is computed correctly but the APF/PID may not react (they use the drone's own velocity for damping).
3. **Single front-facing camera**: Side and rear obstacles are invisible.
4. **Vertical CBF not implemented**: Flying into a ceiling or floor is not protected.
5. **Pipeline latency unmodeled**: Camera → detection → depth → PID is likely 100–400 ms. At 3 m/s cruise this means the drone travels 0.3–1.2 m before any correction. The CBF's `ttc_min_s = 1.5 s` safety margin must be large enough to absorb this delay.
