#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/command_queue.hpp"
#include "../core/messages.hpp"
#include "../utils/logger.hpp"

struct PIDConfig {
    float loop_rate_hz = 20.0f;

    // ── Forward speed PID (TTC-based) ─────────────────────────────────────
    // error = ttc_target_s - ttc_current_s
    // output = scale factor applied to desired forward speed
    float ttc_target_s      = 5.0f;   // desired TTC when approaching
    float fwd_kp            = 0.15f;
    float fwd_ki            = 0.02f;
    float fwd_kd            = 0.05f;
    float fwd_max_integral  = 2.0f;   // anti-windup clamp

    // ── Altitude hold PID ────────────────────────────────────────────────
    // error = alt_desired_m - alt_current_m
    // output = down_m_s correction (negative = up)
    float alt_kp            = 0.8f;
    float alt_ki            = 0.05f;
    float alt_kd            = 0.2f;
    float alt_max_integral  = 1.0f;
    float alt_max_output_m_s = 1.5f;  // clamp vertical correction

    // ── Lateral centre PID ───────────────────────────────────────────────
    // error = 0.5 - centre_x_norm   (positive = obstacle left of centre → go right)
    // output = lateral body-frame velocity correction
    float lat_kp            = 1.2f;
    float lat_ki            = 0.0f;
    float lat_kd            = 0.15f;
    float lat_max_integral  = 1.0f;
    float lat_max_output_m_s = 1.5f;

    // Rate limiting: max velocity change per tick to smooth commands
    float max_delta_speed_per_tick = 0.5f;  // m/s per tick
};

// ── PIDService ────────────────────────────────────────────────────────────────
//
// Runs three PID loops that refine the desired velocity from
// AvoidancePlannerService before passing it on to CommandService.
//
// Loop 1 – Forward speed PID (TTC-based):
//   When an obstacle is approaching, scale down forward speed so we maintain
//   a comfortable time-to-collision (ttc_target_s).
//
// Loop 2 – Altitude hold PID:
//   While flying, maintain the target altitude.  Any deviation caused by
//   offboard mode drift is corrected via the down_m_s channel.
//
// Loop 3 – Lateral offset PID:
//   During dodging, gently push the flight path away from the obstacle's
//   centre pixel so the drone goes around rather than grazing it.
//
// Inputs:
//   ch_desired_vel – VelocityCommand from AvoidancePlannerService
//   ch_ekf         – EKFSnapshot for altitude and velocity feedback
//   ch_depth       – DepthEstimate for TTC feedback
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

    PIDConfig                       cfg_;
    DataChannel<VelocityCommand>&   ch_desired_vel_;
    DataChannel<EKFSnapshot>&       ch_ekf_;
    DataChannel<DepthEstimate>&     ch_depth_;
    CommandQueue<VelocityCommand>&  cq_corrected_;
    Logger&                         logger_;

    float target_alt_m_{5.0f};

    // PID integrators and previous errors
    float fwd_integral_{0.0f},  fwd_prev_err_{0.0f};
    float alt_integral_{0.0f},  alt_prev_err_{0.0f};
    float lat_integral_{0.0f},  lat_prev_err_{0.0f};

    // Previous output for rate limiting
    VelocityCommand prev_output_{};
};
