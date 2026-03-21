#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/messages.hpp"
#include "../navigation/rotation_tracker.hpp"
#include "../utils/logger.hpp"

struct AvoidancePlannerConfig {
    // Poll rate
    float loop_rate_hz      = 20.0f;

    // TTC thresholds (seconds)
    float ttc_clear_s       = 10.0f;  // above this → CLEAR (full cruise speed)
    float ttc_warn_s        = 6.0f;   // between warn and clear → MONITORING
    float ttc_brake_s       = 3.0f;   // between brake and warn → BRAKING
    float ttc_dodge_s       = 2.0f;   // below this → DODGING / CLIMBING

    // Speeds (m/s)
    float cruise_speed_m_s  = 3.0f;
    float dodge_speed_m_s   = 2.0f;
    float ascent_speed_m_s  = 1.0f;
    float max_forward_speed = 5.0f;
    float max_lateral_speed = 3.0f;

    // Lateral offset PID (centre_x_norm error → lateral correction)
    float lateral_kp        = 1.5f;   // proportional gain
    float lateral_ki        = 0.0f;   // integral gain (small, mostly P control)
    float lateral_kd        = 0.2f;   // derivative gain

    // Sector selection: sectors 0-2=left, 3-4=centre, 5-7=right
    int   left_sector_end   = 2;
    int   right_sector_start = 5;
};

// Avoidance state machine states
enum class AvoidanceState {
    CLEAR,
    MONITORING,
    BRAKING,
    DODGING_LEFT,
    DODGING_RIGHT,
    CLIMBING,
};

// ── AvoidancePlannerService ───────────────────────────────────────────────────
//
// Polls at loop_rate_hz for the latest SyncedObservation, runs the avoidance
// state machine, and publishes a VelocityCommand in NED.
//
// Key improvements over the old AvoidanceStrategy:
//   - Uses TTC (time-to-collision) from DepthService instead of raw coverage ratio
//   - Body→NED rotation via full 3×3 quaternion-derived matrix (RotationTracker)
//   - Dodge direction chosen from the sector_histogram (go towards the emptier half)
//   - Lateral offset PID to centre the flight path relative to the obstacle edge
//   - Avoidance can be enabled/disabled at runtime via enable()/disable()

class AvoidancePlannerService : public ServiceBase {
public:
    AvoidancePlannerService(const AvoidancePlannerConfig&  cfg,
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

    VelocityCommand compute(const SyncedObservation& obs);

    // Setpoint builders (all output in NED)
    VelocityCommand build_forward(const EKFSnapshot& ekf, float speed);
    VelocityCommand build_lateral_dodge(const EKFSnapshot& ekf,
                                        float speed, bool left);
    VelocityCommand build_hold(const EKFSnapshot& ekf);
    VelocityCommand build_climb(const EKFSnapshot& ekf);

    // Select dodge direction from the sector histogram.
    // Returns true=left, false=right.
    bool select_dodge_direction(const std::array<float, 8>& histogram) const;

    AvoidancePlannerConfig         cfg_;
    DataChannel<SyncedObservation>& ch_synced_;
    DataChannel<VelocityCommand>&   ch_desired_vel_;
    Logger&                         logger_;

    RotationTracker rotation_;

    AvoidanceState  state_{AvoidanceState::CLEAR};
    AvoidanceState  prev_state_{AvoidanceState::CLEAR};

    bool dodge_dir_latched_{false};
    bool dodge_left_{true};

    std::atomic<bool> enabled_{false};

    // Lateral offset PID state
    float lateral_integral_{0.0f};
    float lateral_prev_error_{0.0f};
};
