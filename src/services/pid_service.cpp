#include "pid_service.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>

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

    const float dt = 1.0f / cfg_.loop_rate_hz;
    const auto  period = std::chrono::duration<double>(dt);

    while (running_) {
        auto tick_start = std::chrono::steady_clock::now();

        VelocityCommand desired = ch_desired_vel_.latest();
        EKFSnapshot     ekf     = ch_ekf_.latest();
        DepthEstimate   depth   = ch_depth_.latest();

        VelocityCommand output  = desired;   // start from desired, then correct
        output.timestamp_us     = now_us();
        output.yaw_deg          = desired.yaw_deg;

        // ── PID 1: Forward speed (TTC-based) ─────────────────────────────
        //
        // Scale the forward component of the desired velocity.
        // Only active when an obstacle is detected (depth.confidence > threshold).

        if (depth.confidence > 0.2f && depth.ttc_s < cfg_.ttc_target_s * 1.5f) {
            float fwd_err = depth.ttc_s - cfg_.ttc_target_s;   // positive = too close

            fwd_integral_ += fwd_err * dt;
            fwd_integral_  = clamp(fwd_integral_,
                                   -cfg_.fwd_max_integral, cfg_.fwd_max_integral);

            float fwd_deriv = (fwd_err - fwd_prev_err_) / dt;
            fwd_prev_err_   = fwd_err;

            float scale = 1.0f + cfg_.fwd_kp * fwd_err
                                + cfg_.fwd_ki * fwd_integral_
                                + cfg_.fwd_kd * fwd_deriv;
            scale = clamp(scale, 0.0f, 1.5f);   // never reverse or over-accelerate

            output.north_m_s *= scale;
            output.east_m_s  *= scale;
        } else {
            fwd_integral_ = 0.0f;
            fwd_prev_err_ = 0.0f;
        }

        // ── PID 2: Altitude hold ─────────────────────────────────────────
        //
        // Correct down_m_s so the drone holds target_alt_m_ during manoeuvres.

        {
            float alt_err     = target_alt_m_ - ekf.alt_rel_m;   // positive = too low
            alt_integral_    += alt_err * dt;
            alt_integral_     = clamp(alt_integral_,
                                      -cfg_.alt_max_integral, cfg_.alt_max_integral);
            float alt_deriv   = (alt_err - alt_prev_err_) / dt;
            alt_prev_err_     = alt_err;

            float correction  = cfg_.alt_kp * alt_err
                              + cfg_.alt_ki * alt_integral_
                              + cfg_.alt_kd * alt_deriv;
            correction        = clamp(correction,
                                      -cfg_.alt_max_output_m_s, cfg_.alt_max_output_m_s);

            // In NED, down_m_s positive = descend.  altitude correction:
            //   if alt_err > 0 (too low) → climb → down_m_s negative
            output.down_m_s = -correction;
        }

        // ── PID 3: Lateral offset (centre the path on the clear side) ────
        //
        // centre_x_norm = 0.5 means obstacle is dead centre.
        // error > 0 means obstacle is right of centre → nudge cmd left (negative right)

        if (depth.confidence > 0.2f) {
            // Use the sector histogram gradient as the lateral error proxy.
            // Left sector mean vs right sector mean: push towards emptier side.
            float left_sum = 0.0f, right_sum = 0.0f;
            for (int i = 0; i < 4; i++)
                left_sum  += depth.sector_histogram[static_cast<size_t>(i)];
            for (int i = 4; i < 8; i++)
                right_sum += depth.sector_histogram[static_cast<size_t>(i)];

            // Normalised asymmetry: +1 = all danger on right, -1 = all danger on left
            float total = left_sum + right_sum;
            float lat_err = (total > 1e-4f) ? (right_sum - left_sum) / total : 0.0f;

            lat_integral_  += lat_err * dt;
            lat_integral_   = clamp(lat_integral_,
                                    -cfg_.lat_max_integral, cfg_.lat_max_integral);
            float lat_deriv = (lat_err - lat_prev_err_) / dt;
            lat_prev_err_   = lat_err;

            float lat_corr = cfg_.lat_kp * lat_err
                           + cfg_.lat_ki * lat_integral_
                           + cfg_.lat_kd * lat_deriv;
            lat_corr = clamp(lat_corr, -cfg_.lat_max_output_m_s, cfg_.lat_max_output_m_s);

            // lat_corr > 0 means push right (away from right danger).
            // In NED east is right.  We don't know current yaw here easily,
            // so we add it as an east adjustment – small correction, acceptable.
            output.east_m_s += lat_corr;
        } else {
            lat_integral_ = 0.0f;
            lat_prev_err_ = 0.0f;
        }

        // ── Rate limiting ─────────────────────────────────────────────────
        //
        // Prevent step-changes in commands that could cause jerky flight.

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
