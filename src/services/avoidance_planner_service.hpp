#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/messages.hpp"
#include "../navigation/rotation_tracker.hpp"
#include "../utils/logger.hpp"

// ── AvoidancePlannerConfig ────────────────────────────────────────────────────
//
// Configuration for the APF-based guidance layer.
// Replaces the old discrete state-machine config.

struct AvoidancePlannerConfig {
    float loop_rate_hz            = 20.0f;

    // Maximum age of a SyncedObservation before it is treated as stale.
    float obs_max_age_s           = 0.5f;

    // ── Artificial Potential Fields ───────────────────────────────────────
    // Attractive gain: equals the cruise speed (m/s).
    // This is the constant forward "pull" toward the mission direction.
    float k_att                   = 3.0f;

    // Repulsive gain: scales the Khatib TTC-space potential.
    // Larger = earlier and stronger reaction to obstacles.
    float k_rep                   = 20.0f;

    // Velocity damping: multiplied by current body velocity and subtracted
    // from the total force. Reduces oscillation in the potential field.
    float k_damp                  = 0.3f;

    // TTC influence radius (s): obstacle repulsion activates when TTC < this.
    float ttc_influence_s         = 10.0f;

    // Camera horizontal field of view (degrees).
    // Used to map sector index 0..7 to bearing angle in the body frame.
    float camera_hfov_deg         = 60.0f;

    // Maximum horizontal output speed (m/s).
    float max_speed_m_s           = 5.0f;

    // Ascent speed for emergency climb (m/s, applied as -down_m_s in NED).
    float ascent_speed_m_s        = 1.0f;

    // TTC below which the emergency climb override fires regardless of APF.
    float ttc_emergency_s         = 1.5f;

    // ── Stuck detection / escape ──────────────────────────────────────────
    // When body speed stays below this threshold while an obstacle is active,
    // the stuck counter increments. After stuck_timeout_ticks the planner
    // injects alternating lateral kicks to escape local minima.
    float stuck_vel_threshold_m_s = 0.2f;
    int   stuck_timeout_ticks     = 20;    // 1 s at 20 Hz
    float stuck_kick_speed_m_s    = 0.8f;
};

// Simplified avoidance state for logging and external monitoring.
enum class AvoidanceState {
    CLEAR,     // no obstacle, attractive force only
    ACTIVE,    // APF repulsion active
    CLIMBING,  // emergency climb (TTC critically low)
};

// ── AvoidancePlannerService ───────────────────────────────────────────────────
//
// Polls at loop_rate_hz for the latest SyncedObservation and runs an
// Artificial Potential Fields guidance law to produce a VelocityCommand in NED.
//
// Architecture:
//   Attractive force  – constant forward pull at k_att (cruise speed).
//   Repulsive force   – Khatib TTC-space potential, directed away from the
//                       obstacle centroid derived from the sector histogram.
//   Damping force     – viscous friction on current body velocity to reduce
//                       oscillation near equilibrium points.
//   Emergency climb   – overrides APF when TTC < ttc_emergency_s (hard safety).
//   Stuck escape      – lateral kicks when the drone stalls in a local minimum.
//
// Downstream: publishes VelocityCommand to ch_desired_vel_.
// PIDService then applies altitude correction and the CBF safety filter.

class AvoidancePlannerService : public ServiceBase {
public:
    AvoidancePlannerService(const AvoidancePlannerConfig&   cfg,
                             DataChannel<SyncedObservation>& ch_synced,
                             DataChannel<VelocityCommand>&   ch_desired_vel,
                             Logger&                         logger);

    const char* name() const override { return "AvoidancePlannerService"; }

    void enable()  { enabled_ = true; }
    void disable() { enabled_ = false; }

    AvoidanceState current_state() const { return state_; }
    const char*    state_name()    const;

private:
    void run() override;

    // Main APF computation – returns a VelocityCommand in NED.
    VelocityCommand compute(const SyncedObservation& obs);

    // Compute the weighted obstacle bearing angle (radians, body frame,
    // positive = rightward) from the 8-sector edge density histogram.
    float compute_obstacle_angle(const std::array<float, 8>& histogram) const;

    // Emergency climb setpoint (zero horizontal, ascend at ascent_speed_m_s).
    VelocityCommand build_climb(const EKFSnapshot& ekf) const;

    AvoidancePlannerConfig          cfg_;
    DataChannel<SyncedObservation>& ch_synced_;
    DataChannel<VelocityCommand>&   ch_desired_vel_;
    Logger&                         logger_;

    RotationTracker rotation_;

    AvoidanceState state_{AvoidanceState::CLEAR};
    AvoidanceState prev_state_{AvoidanceState::CLEAR};

    std::atomic<bool> enabled_{false};

    // Stuck detection
    int  stuck_ticks_{0};
    bool stuck_kick_left_{true};
};
