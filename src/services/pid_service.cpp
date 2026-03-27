#include "pid_service.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

PIDService::PIDService(const PIDConfig&                cfg,
                        DataChannel<VelocityCommand>&   ch_desired_vel,
                        DataChannel<EKFSnapshot>&        ch_ekf,
                        DataChannel<DepthEstimate>&      ch_depth,
                        CommandQueue<VelocityCommand>&   cq_corrected,
                        Logger&                          logger)
    : cfg_(cfg)
    , ch_desired_vel_(ch_desired_vel)
    , ch_ekf_(ch_ekf)
    , ch_depth_(ch_depth)
    , cq_corrected_(cq_corrected)
    , logger_(logger)
{}

static float clamp(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void PIDService::run() {
    logger_.info("[PIDService] Started");

    const auto period = std::chrono::duration<double>(1.0 / cfg_.loop_rate_hz);

    while (running_) {
        auto tick_start = std::chrono::steady_clock::now();

        // ── Measure actual elapsed dt ─────────────────────────────────────
        // Using measured dt instead of a fixed 1/rate value ensures that
        // integrators and derivatives remain accurate under OS scheduling
        // jitter (typically ±2–5 ms at 20 Hz).
        float dt;
        if (first_tick_) {
            dt          = 1.0f / cfg_.loop_rate_hz;
            first_tick_ = false;
        } else {
            dt = std::chrono::duration<float>(tick_start - prev_tick_time_).count();
            // Clamp to reject absurd values from preemption or debugging.
            dt = clamp(dt, 0.005f, 0.2f);
        }
        prev_tick_time_ = tick_start;

        VelocityCommand desired = ch_desired_vel_.latest();
        EKFSnapshot     ekf     = ch_ekf_.latest();
        DepthEstimate   depth   = ch_depth_.latest();

        rotation_.update(ekf);

        // Initialise prev_alt_m_ on first real tick to avoid a derivative spike.
        if (alt_deriv_filt_ == 0.0f && alt_integral_ == 0.0f) {
            prev_alt_m_ = ekf.alt_rel_m;
        }

        VelocityCommand output  = desired;
        output.timestamp_us     = now_us();
        output.yaw_deg          = desired.yaw_deg;

        // ── PID 1: Forward speed (TTC-based) ─────────────────────────────
        // Scales the horizontal component of the desired velocity to maintain
        // a comfortable time-to-collision.  Only active when an obstacle is
        // detected with sufficient confidence and TTC is within the working
        // range.
        if (depth.confidence > 0.2f && depth.ttc_s < cfg_.ttc_target_s * 1.5f) {
            float fwd_err = depth.ttc_s - cfg_.ttc_target_s;

            // Filtered derivative on error (ttc_target is constant, so no kick).
            float fwd_raw_deriv = (fwd_err - fwd_prev_err_) / dt;
            fwd_prev_err_ = fwd_err;
            float alpha_fwd  = cfg_.fwd_deriv_tau / (cfg_.fwd_deriv_tau + dt);
            fwd_deriv_filt_ = alpha_fwd * fwd_deriv_filt_
                            + (1.0f - alpha_fwd) * fwd_raw_deriv;

            // Compute scale before integration to implement conditional
            // anti-windup: only integrate when output is not saturated, or
            // when the integrator is helping to desaturate.
            float scale_pre = 1.0f + cfg_.fwd_kp * fwd_err
                                   + cfg_.fwd_ki * fwd_integral_
                                   + cfg_.fwd_kd * fwd_deriv_filt_;
            bool sat  = (scale_pre <= 0.0f || scale_pre >= 1.5f);
            bool hlps = (scale_pre > 1.5f && fwd_err < 0.0f) ||
                        (scale_pre < 0.0f  && fwd_err > 0.0f);
            if (!sat || hlps) {
                fwd_integral_ += fwd_err * dt;
                fwd_integral_  = clamp(fwd_integral_,
                                       -cfg_.fwd_max_integral, cfg_.fwd_max_integral);
            }

            float scale = 1.0f + cfg_.fwd_kp * fwd_err
                               + cfg_.fwd_ki * fwd_integral_
                               + cfg_.fwd_kd * fwd_deriv_filt_;
            scale = clamp(scale, 0.0f, 1.5f);

            output.north_m_s *= scale;
            output.east_m_s  *= scale;
        } else {
            // Reset when obstacle not active so previous integral doesn't
            // cause an initial burst when the next obstacle appears.
            fwd_integral_   = 0.0f;
            fwd_prev_err_   = 0.0f;
            fwd_deriv_filt_ = 0.0f;
        }

        // ── PID 2: Altitude hold ──────────────────────────────────────────
        // Corrects down_m_s to maintain target_alt_m_ throughout all
        // manoeuvres.
        //
        // Derivative-on-measurement: d/dt(-altitude) rather than d/dt(error).
        // This eliminates derivative kick when set_target_altitude() is called
        // (a step change in the target does not cause a spike in the output).
        //
        // First-order low-pass on the derivative suppresses noise from the
        // EKF altitude estimate.
        //
        // Conditional anti-windup: the integrator freezes when the output
        // is saturated and would only make saturation worse.
        {
            float alt_err = target_alt_m_ - ekf.alt_rel_m;

            // Derivative on measurement (negative because d(error)/dt = -d(alt)/dt)
            float raw_deriv = -(ekf.alt_rel_m - prev_alt_m_) / dt;
            prev_alt_m_     = ekf.alt_rel_m;

            float alpha_alt  = cfg_.alt_deriv_tau / (cfg_.alt_deriv_tau + dt);
            alt_deriv_filt_ = alpha_alt * alt_deriv_filt_
                            + (1.0f - alpha_alt) * raw_deriv;

            // Compute output before integration for conditional anti-windup.
            float raw_correction = cfg_.alt_kp * alt_err
                                 + cfg_.alt_ki * alt_integral_
                                 + cfg_.alt_kd * alt_deriv_filt_;

            bool sat  = (raw_correction >=  cfg_.alt_max_output_m_s ||
                         raw_correction <= -cfg_.alt_max_output_m_s);
            bool hlps = (raw_correction >  cfg_.alt_max_output_m_s && alt_err < 0.0f) ||
                        (raw_correction < -cfg_.alt_max_output_m_s && alt_err > 0.0f);
            if (!sat || hlps) {
                alt_integral_ += alt_err * dt;
                alt_integral_  = clamp(alt_integral_,
                                       -cfg_.alt_max_integral, cfg_.alt_max_integral);
            }

            float correction = clamp(raw_correction,
                                     -cfg_.alt_max_output_m_s, cfg_.alt_max_output_m_s);

            // NED convention: negative down_m_s = climb.
            // alt_err > 0 (below target) → climb → down_m_s negative.
            output.down_m_s = -correction;
        }

        // ── Rate limiting ─────────────────────────────────────────────────
        // Prevents step-changes in the command that could cause jerky flight.
        auto rate_limit = [&](float curr, float prev) -> float {
            float delta = curr - prev;
            float max_d = cfg_.max_delta_speed_per_tick;
            if (delta >  max_d) return prev + max_d;
            if (delta < -max_d) return prev - max_d;
            return curr;
        };

        output.north_m_s = rate_limit(output.north_m_s, prev_output_.north_m_s);
        output.east_m_s  = rate_limit(output.east_m_s,  prev_output_.east_m_s);
        output.down_m_s  = rate_limit(output.down_m_s,  prev_output_.down_m_s);

        prev_output_ = output;

        // ── CBF safety filter ─────────────────────────────────────────────
        // Applied after all PID corrections and rate limiting.
        // This is the last line of defence before the command goes to the
        // flight controller – it mathematically guarantees TTC >= ttc_min_s.
        if (cfg_.cbf_enabled) {
            output = apply_cbf(output, depth, ekf);
        }

        cq_corrected_.push(output);

        // ── Loop rate enforcement ─────────────────────────────────────────
        auto elapsed   = std::chrono::steady_clock::now() - tick_start;
        auto sleep_for = period - elapsed;
        if (sleep_for.count() > 0) {
            std::this_thread::sleep_for(sleep_for);
        }
    }

    logger_.info("[PIDService] Stopped");
}

// ── apply_cbf ─────────────────────────────────────────────────────────────────
//
// Control Barrier Function safety filter.
//
// Safety function: h(x) = TTC - ttc_min ≥ 0
// CBF constraint:  ∇h · u + γ · h ≥ 0
//
// ∇h points in the direction that increases h (i.e. away from the obstacle).
// When the constraint is violated the velocity is projected onto the safe
// halfspace: u_safe = u + λ · ∇h, where λ = -(∇h · u + γ·h) / ||∇h||².
// This is the minimum-norm modification – it changes the command as little
// as possible while restoring safety.

VelocityCommand PIDService::apply_cbf(const VelocityCommand& u,
                                       const DepthEstimate&   depth,
                                       const EKFSnapshot&     /*ekf*/) {
    float h = depth.ttc_s - cfg_.cbf_ttc_min_s;

    // Only engage near the safety boundary and with reliable sensing.
    if (h > cfg_.cbf_activate_h || depth.confidence < 0.2f) {
        return u;
    }

    // Convert NED command to body frame so we can reason about obstacle direction.
    float u_fwd, u_right, u_dn;
    rotation_.ned_to_body(u.north_m_s, u.east_m_s, u.down_m_s,
                          u_fwd, u_right, u_dn);

    // ∇h in body frame: direction away from the obstacle.
    // The obstacle bearing is computed from the sector histogram centroid;
    // ∇h is the opposite direction (flee direction).
    float obstacle_angle = compute_obstacle_angle(depth.sector_histogram);
    float grad_h_x = -std::cos(obstacle_angle);   // body frame X (forward)
    float grad_h_y = -std::sin(obstacle_angle);   // body frame Y (right)

    // Guard against degenerate histogram (all zeros → obstacle_angle = 0,
    // grad_h = [-1, 0] which is fine: push backward).
    float grad_norm = std::hypot(grad_h_x, grad_h_y);
    if (grad_norm < 1e-4f) return u;
    grad_h_x /= grad_norm;
    grad_h_y /= grad_norm;

    // CBF constraint value: ∇h · u + γ · h
    float dot     = grad_h_x * u_fwd + grad_h_y * u_right;
    float cbf_val = dot + cfg_.cbf_gamma * h;

    // Constraint satisfied – no modification needed.
    if (cbf_val >= 0.0f) return u;

    // Constraint violated: project u onto the safe halfspace.
    // λ = -cbf_val / ||∇h||² = -cbf_val  (∇h is already normalised).
    float lambda = -cbf_val;
    u_fwd   += lambda * grad_h_x;
    u_right += lambda * grad_h_y;

    logger_.debug("[PIDService] CBF: h=", h,
                  " val=", cbf_val, " λ=", lambda);

    // Rotate corrected body velocity back to NED.
    // The altitude channel (down_m_s) is passed through unchanged – the
    // CBF only constrains horizontal approach speed.
    VelocityCommand u_safe = u;
    float north, east, down_dummy;
    rotation_.body_to_ned(u_fwd, u_right, 0.0f, north, east, down_dummy);
    u_safe.north_m_s = north;
    u_safe.east_m_s  = east;

    return u_safe;
}

// ── compute_obstacle_angle ────────────────────────────────────────────────────

float PIDService::compute_obstacle_angle(
        const std::array<float, 8>& histogram) const {
    const float sector_rad =
        (cfg_.camera_hfov_deg * static_cast<float>(M_PI) / 180.0f) / 8.0f;

    float weighted_angle = 0.0f;
    float total_density  = 0.0f;

    for (int i = 0; i < 8; i++) {
        float angle = (static_cast<float>(i) - 3.5f) * sector_rad;
        weighted_angle += angle * histogram[static_cast<size_t>(i)];
        total_density  += histogram[static_cast<size_t>(i)];
    }

    return (total_density > 1e-4f) ? weighted_angle / total_density : 0.0f;
}
