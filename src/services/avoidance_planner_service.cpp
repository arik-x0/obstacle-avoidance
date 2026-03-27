#include "avoidance_planner_service.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std::chrono_literals;

AvoidancePlannerService::AvoidancePlannerService(
        const AvoidancePlannerConfig&   cfg,
        DataChannel<SyncedObservation>& ch_synced,
        DataChannel<VelocityCommand>&   ch_desired_vel,
        Logger&                         logger)
    : cfg_(cfg)
    , ch_synced_(ch_synced)
    , ch_desired_vel_(ch_desired_vel)
    , logger_(logger)
{}

// ── run ───────────────────────────────────────────────────────────────────────

void AvoidancePlannerService::run() {
    logger_.info("[AvoidancePlannerService] Started (APF guidance)");

    const auto period = std::chrono::duration<double>(1.0 / cfg_.loop_rate_hz);

    while (running_) {
        auto tick_start = std::chrono::steady_clock::now();

        if (enabled_) {
            SyncedObservation obs = ch_synced_.latest();

            const uint64_t max_age_us =
                static_cast<uint64_t>(cfg_.obs_max_age_s * 1e6f);
            const bool stale = (now_us() - obs.timestamp_us) > max_age_us;

            VelocityCommand cmd{};
            cmd.timestamp_us = now_us();

            if (stale) {
                if (state_ != AvoidanceState::CLEAR) {
                    logger_.warn("[AvoidancePlanner] Stale observation – coasting forward");
                    state_ = AvoidanceState::CLEAR;
                }
                // On stale data coast forward at half cruise speed.
                float north, east, down_dummy;
                rotation_.body_to_ned(cfg_.k_att * 0.5f, 0.0f, 0.0f,
                                      north, east, down_dummy);
                cmd.north_m_s = north;
                cmd.east_m_s  = east;
            } else {
                cmd = compute(obs);
            }

            ch_desired_vel_.publish(cmd);
        } else {
            // Avoidance disabled – publish zero velocity (hold).
            VelocityCommand hold{};
            hold.timestamp_us = now_us();
            ch_desired_vel_.publish(hold);
        }

        auto elapsed   = std::chrono::steady_clock::now() - tick_start;
        auto sleep_for = period - elapsed;
        if (sleep_for.count() > 0) {
            std::this_thread::sleep_for(sleep_for);
        }
    }

    logger_.info("[AvoidancePlannerService] Stopped");
}

// ── compute ───────────────────────────────────────────────────────────────────

VelocityCommand AvoidancePlannerService::compute(const SyncedObservation& obs) {
    rotation_.update(obs.ekf);

    const auto& det   = obs.detection.result;
    const auto& depth = obs.depth;

    VelocityCommand cmd{};
    cmd.timestamp_us = now_us();
    cmd.yaw_deg      = obs.ekf.yaw_deg;

    const bool obstacle_active = det.obstacle_detected && depth.confidence > 0.2f;

    // ── Emergency climb override ──────────────────────────────────────────
    // When TTC is critically low the APF may not react fast enough.
    // Override with a direct climb command and let the obstacle pass overhead.
    if (obstacle_active && depth.ttc_s < cfg_.ttc_emergency_s) {
        if (state_ != AvoidanceState::CLIMBING) {
            logger_.warn("[Planner] EMERGENCY CLIMB – TTC=", depth.ttc_s, " s");
            state_ = AvoidanceState::CLIMBING;
        }
        prev_state_ = state_;
        stuck_ticks_ = 0;
        return build_climb(obs.ekf);
    }

    // ── Attractive force (body frame) ─────────────────────────────────────
    // Constant forward pull at cruise speed (k_att).  The goal is always
    // "ahead" since this system has no global waypoint.
    float f_att_x = cfg_.k_att;
    float f_att_y = 0.0f;

    // ── Repulsive force (body frame, Khatib TTC-space potential) ──────────
    // U_rep = (1/2) * k_rep * (1/TTC - 1/TTC_0)²
    // F_rep = k_rep * (1/TTC - 1/TTC_0) * (1/TTC²)  directed away from obstacle.
    float f_rep_x = 0.0f;
    float f_rep_y = 0.0f;

    float ttc = obstacle_active ? depth.ttc_s : (cfg_.ttc_influence_s + 1.0f);

    if (obstacle_active && ttc < cfg_.ttc_influence_s && ttc > 0.01f) {
        float inv_ttc  = 1.0f / ttc;
        float inv_ttc0 = 1.0f / cfg_.ttc_influence_s;
        float f_rep_mag = cfg_.k_rep * (inv_ttc - inv_ttc0) * (inv_ttc * inv_ttc);

        // Weight repulsion by sensor confidence to reduce noise-driven reactions.
        f_rep_mag *= depth.confidence;

        // Obstacle centroid bearing from the sector histogram.
        float obstacle_angle = compute_obstacle_angle(depth.sector_histogram);

        // Flee direction is directly opposite the obstacle bearing.
        f_rep_x = -std::cos(obstacle_angle) * f_rep_mag;
        f_rep_y = -std::sin(obstacle_angle) * f_rep_mag;

        state_ = AvoidanceState::ACTIVE;
    } else {
        state_       = AvoidanceState::CLEAR;
        stuck_ticks_ = 0;
    }

    // ── Velocity damping (body frame) ─────────────────────────────────────
    // Acts as viscous friction: F_damp = -k_damp * v_body.
    // Prevents oscillation at the attractive/repulsive equilibrium point.
    float vel_fwd, vel_right, vel_dn;
    rotation_.ned_to_body(obs.ekf.vel_north, obs.ekf.vel_east, obs.ekf.vel_down,
                          vel_fwd, vel_right, vel_dn);

    float f_damp_x = -cfg_.k_damp * vel_fwd;
    float f_damp_y = -cfg_.k_damp * vel_right;

    // ── Combine all forces ────────────────────────────────────────────────
    float f_total_x = f_att_x + f_rep_x + f_damp_x;
    float f_total_y = f_att_y + f_rep_y + f_damp_y;

    // ── Stuck detection and lateral escape kick ───────────────────────────
    // A local minimum (APF forces cancel) manifests as near-zero body speed
    // while an obstacle is active.  Inject alternating lateral kicks to escape.
    float vel_horiz = std::hypot(vel_fwd, vel_right);
    if (obstacle_active && vel_horiz < cfg_.stuck_vel_threshold_m_s) {
        stuck_ticks_++;
    } else {
        stuck_ticks_ = 0;
    }

    if (stuck_ticks_ > cfg_.stuck_timeout_ticks) {
        // Alternate kick direction each timeout period.
        if (stuck_ticks_ % cfg_.stuck_timeout_ticks == 0) {
            stuck_kick_left_ = !stuck_kick_left_;
        }
        float kick = stuck_kick_left_
                     ?  cfg_.stuck_kick_speed_m_s
                     : -cfg_.stuck_kick_speed_m_s;
        f_total_y += kick;
        logger_.debug("[Planner] Stuck escape kick ",
                      stuck_kick_left_ ? "LEFT" : "RIGHT",
                      "  ticks=", stuck_ticks_);
    }

    // ── Clamp combined force to max horizontal speed ──────────────────────
    float f_mag = std::hypot(f_total_x, f_total_y);
    if (f_mag > cfg_.max_speed_m_s) {
        float scale = cfg_.max_speed_m_s / f_mag;
        f_total_x  *= scale;
        f_total_y  *= scale;
    }

    // ── Log state transitions ─────────────────────────────────────────────
    if (state_ != prev_state_) {
        logger_.info("[Planner] → ", state_name(),
                     "  TTC=", ttc, " s",
                     "  conf=", depth.confidence,
                     "  |F|=", f_mag);
        prev_state_ = state_;
    }

    // ── Rotate body frame → NED ───────────────────────────────────────────
    // The down component is zero here; altitude is corrected by PIDService.
    float ned_down_dummy;
    rotation_.body_to_ned(f_total_x, f_total_y, 0.0f,
                          cmd.north_m_s, cmd.east_m_s, ned_down_dummy);
    cmd.down_m_s = 0.0f;

    return cmd;
}

// ── compute_obstacle_angle ────────────────────────────────────────────────────

float AvoidancePlannerService::compute_obstacle_angle(
        const std::array<float, 8>& histogram) const {
    // Map sector index i ∈ [0,7] to horizontal bearing in body frame.
    // Sector 0 = left edge, sector 7 = right edge.
    // Angle convention: positive = rightward (body +Y axis).
    const float sector_rad =
        (cfg_.camera_hfov_deg * static_cast<float>(M_PI) / 180.0f) / 8.0f;

    float weighted_angle = 0.0f;
    float total_density  = 0.0f;

    for (int i = 0; i < 8; i++) {
        float angle = (static_cast<float>(i) - 3.5f) * sector_rad;
        weighted_angle += angle * histogram[static_cast<size_t>(i)];
        total_density  += histogram[static_cast<size_t>(i)];
    }

    // If histogram is empty (no confident detection) return 0 (straight ahead).
    return (total_density > 1e-4f) ? weighted_angle / total_density : 0.0f;
}

// ── build_climb ───────────────────────────────────────────────────────────────

VelocityCommand AvoidancePlannerService::build_climb(const EKFSnapshot& ekf) const {
    VelocityCommand cmd{};
    cmd.timestamp_us = now_us();
    cmd.yaw_deg      = ekf.yaw_deg;
    cmd.down_m_s     = -cfg_.ascent_speed_m_s;   // negative = up in NED
    return cmd;
}

// ── state_name ────────────────────────────────────────────────────────────────

const char* AvoidancePlannerService::state_name() const {
    switch (state_) {
        case AvoidanceState::CLEAR:    return "CLEAR";
        case AvoidanceState::ACTIVE:   return "ACTIVE";
        case AvoidanceState::CLIMBING: return "CLIMBING";
    }
    return "UNKNOWN";
}
