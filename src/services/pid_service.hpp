#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/command_queue.hpp"
#include "../core/messages.hpp"
#include "../navigation/rotation_tracker.hpp"
#include "../utils/logger.hpp"

struct PIDConfig {
    float loop_rate_hz = 20.0f;

    // ── Forward speed PID (TTC-based) ─────────────────────────────────────
    // error  = ttc_current - ttc_target   (positive = obstacle too close)
    // output = scale factor on the desired horizontal velocity [0, 1.5]
    float ttc_target_s      = 5.0f;
    float fwd_kp            = 0.15f;
    float fwd_ki            = 0.02f;
    float fwd_kd            = 0.05f;
    float fwd_max_integral  = 2.0f;
    float fwd_deriv_tau     = 0.15f;   // first-order low-pass τ for derivative (s)

    // ── Altitude hold PID ────────────────────────────────────────────────
    // error  = target_alt - current_alt  (positive = too low → climb)
    // output = down_m_s correction (negative = up in NED)
    // Derivative is computed on the measurement (not the error) to avoid
    // derivative kick when the target altitude is updated by FlightManager.
    float alt_kp            = 0.8f;
    float alt_ki            = 0.05f;
    float alt_kd            = 0.2f;
    float alt_max_integral  = 1.0f;
    float alt_max_output_m_s = 1.5f;
    float alt_deriv_tau     = 0.1f;    // first-order low-pass τ for derivative (s)

    // ── Rate limiting ─────────────────────────────────────────────────────
    // Prevent step-changes in the velocity command (m/s per tick).
    float max_delta_speed_per_tick = 0.5f;

    // ── Control Barrier Function (CBF) safety filter ──────────────────────
    // The CBF sits after all PID corrections and rate limiting.
    // Safety function: h = TTC - cbf_ttc_min_s >= 0
    // CBF constraint:  ∇h · u + γ · h >= 0
    // When violated the velocity is minimally projected onto the safe halfspace.
    // This guarantees TTC never falls below cbf_ttc_min_s regardless of what
    // the upstream guidance layer sends.
    bool  cbf_enabled       = true;
    float cbf_ttc_min_s     = 1.5f;   // hard minimum TTC (safety boundary, s)
    float cbf_gamma         = 1.0f;   // class-K function gain (higher = tighter)
    float cbf_activate_h    = 3.0f;   // only engage CBF when h < this value
    float camera_hfov_deg   = 60.0f;  // camera FOV for obstacle angle computation
};

// ── PIDService ────────────────────────────────────────────────────────────────
//
// Runs two PID loops and a CBF safety filter that refine the desired velocity
// from AvoidancePlannerService before passing it to CommandService.
//
// PID 1 – Forward speed (TTC-based):
//   Scales horizontal velocity to maintain a comfortable time-to-collision.
//   Active only when depth.confidence > 0.2 and TTC is near the target.
//
// PID 2 – Altitude hold:
//   Corrects down_m_s to maintain target_alt_m_ during manoeuvres.
//   Uses derivative-on-measurement to avoid derivative kick when the target
//   altitude changes. Uses a first-order low-pass on the derivative to
//   suppress sensor noise. Uses conditional integration (anti-windup).
//
// CBF safety filter:
//   After rate limiting, projects the velocity command onto the safe halfspace
//   defined by the CBF constraint when the safety margin h = TTC - ttc_min
//   is too small. This is the minimum-modification safety guarantee.
//
// Inputs:
//   ch_desired_vel – VelocityCommand from AvoidancePlannerService
//   ch_ekf         – EKFSnapshot for altitude and attitude feedback
//   ch_depth       – DepthEstimate for TTC and sector histogram feedback
//
// Output:
//   cq_corrected   – CommandQueue<VelocityCommand> consumed by CommandService

class PIDService : public ServiceBase {
public:
    PIDService(const PIDConfig&                cfg,
               DataChannel<VelocityCommand>&   ch_desired_vel,
               DataChannel<EKFSnapshot>&        ch_ekf,
               DataChannel<DepthEstimate>&      ch_depth,
               CommandQueue<VelocityCommand>&   cq_corrected,
               Logger&                          logger);

    const char* name() const override { return "PIDService"; }

    // Update the desired altitude target (called by FlightManagerService on takeoff).
    void set_target_altitude(float alt_m) { target_alt_m_ = alt_m; }

private:
    void run() override;

    // CBF safety filter: minimally modifies the NED velocity command so that
    // the safety constraint ∇h·u + γ·h ≥ 0 is satisfied.
    VelocityCommand apply_cbf(const VelocityCommand& u,
                              const DepthEstimate&   depth,
                              const EKFSnapshot&     ekf);

    // Compute the weighted obstacle bearing angle (radians, body frame,
    // positive = rightward) from the 8-sector edge density histogram.
    float compute_obstacle_angle(const std::array<float, 8>& histogram) const;

    PIDConfig                       cfg_;
    DataChannel<VelocityCommand>&   ch_desired_vel_;
    DataChannel<EKFSnapshot>&       ch_ekf_;
    DataChannel<DepthEstimate>&     ch_depth_;
    CommandQueue<VelocityCommand>&  cq_corrected_;
    Logger&                         logger_;

    float target_alt_m_{5.0f};

    // Forward speed PID state
    float fwd_integral_{0.0f};
    float fwd_prev_err_{0.0f};
    float fwd_deriv_filt_{0.0f};   // filtered derivative

    // Altitude hold PID state
    float alt_integral_{0.0f};
    float prev_alt_m_{0.0f};       // previous altitude measurement (derivative-on-measurement)
    float alt_deriv_filt_{0.0f};   // filtered derivative

    // Rate limiting
    VelocityCommand prev_output_{};

    // Timing – actual elapsed dt is measured each tick instead of using a
    // fixed value so that integrators and derivatives are accurate under
    // OS scheduling jitter.
    std::chrono::steady_clock::time_point prev_tick_time_{};
    bool first_tick_{true};

    // Rotation tracker needed by the CBF filter to convert between NED and
    // body frame for the constraint direction computation.
    RotationTracker rotation_;
};
