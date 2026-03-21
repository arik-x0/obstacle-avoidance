#include "avoidance_planner_service.hpp"

#include <thread>
#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm>

using namespace std::chrono_literals;

AvoidancePlannerService::AvoidancePlannerService(
        const AvoidancePlannerConfig&  cfg,
        DataChannel<SyncedObservation>& ch_synced,
        DataChannel<VelocityCommand>&   ch_desired_vel,
        Logger&                         logger)
    : cfg_(cfg)
    , ch_synced_(ch_synced)
    , ch_desired_vel_(ch_desired_vel)
    , logger_(logger)
{}

void AvoidancePlannerService::run() {
    logger_.info("[AvoidancePlannerService] Started");

    const auto period = std::chrono::duration<double>(1.0 / cfg_.loop_rate_hz);

    while (running_) {
        auto tick_start = std::chrono::steady_clock::now();

        if (enabled_) {
            SyncedObservation obs = ch_synced_.latest();
            VelocityCommand   cmd = compute(obs);
            ch_desired_vel_.publish(cmd);
        } else {
            // Avoidance disabled – publish a hold command so PIDService and
            // CommandService know we want zero velocity.
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

// ── Main compute ─────────────────────────────────────────────────────────────

VelocityCommand AvoidancePlannerService::compute(const SyncedObservation& obs) {
    rotation_.update(obs.ekf);

    const auto& det   = obs.detection.result;
    const auto& depth = obs.depth;

    float ttc = depth.ttc_s;
    // If obstacle not actually detected, treat as infinite TTC
    if (!det.obstacle_detected) ttc = cfg_.ttc_clear_s + 1.0f;

    prev_state_ = state_;

    // ── State transitions ─────────────────────────────────────────────────

    switch (state_) {
        case AvoidanceState::CLEAR:
            if (det.obstacle_detected &&
                det.sector == DetectionResult::Sector::CENTER &&
                ttc < cfg_.ttc_warn_s) {
                state_ = AvoidanceState::MONITORING;
            }
            break;

        case AvoidanceState::MONITORING:
            if (!det.obstacle_detected || ttc >= cfg_.ttc_clear_s) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
            } else if (ttc < cfg_.ttc_brake_s) {
                state_ = AvoidanceState::BRAKING;
            }
            break;

        case AvoidanceState::BRAKING:
            if (!det.obstacle_detected || ttc >= cfg_.ttc_warn_s) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
            } else if (ttc < cfg_.ttc_dodge_s) {
                if (!dodge_dir_latched_) {
                    dodge_left_        = select_dodge_direction(depth.sector_histogram);
                    dodge_dir_latched_ = true;
                    rotation_.mark_yaw_start();
                    logger_.info("[Planner] Dodge direction latched: ",
                                 dodge_left_ ? "LEFT" : "RIGHT",
                                 "  TTC=", ttc, " s");
                }
                state_ = dodge_left_ ? AvoidanceState::DODGING_LEFT
                                     : AvoidanceState::DODGING_RIGHT;
            }
            break;

        case AvoidanceState::DODGING_LEFT:
        case AvoidanceState::DODGING_RIGHT:
            if (!det.obstacle_detected || ttc >= cfg_.ttc_clear_s) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
                lateral_integral_  = 0.0f;
                logger_.info("[Planner] Path clear – resuming CLEAR");
            } else if (det.threat() == DetectionResult::Threat::CRITICAL &&
                       ttc < cfg_.ttc_dodge_s * 0.5f) {
                state_ = AvoidanceState::CLIMBING;
                logger_.warn("[Planner] Dodge insufficient – climbing");
            }
            break;

        case AvoidanceState::CLIMBING:
            if (!det.obstacle_detected || ttc >= cfg_.ttc_warn_s) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
                logger_.info("[Planner] Obstacle cleared above – CLEAR");
            }
            break;
    }

    if (state_ != prev_state_) {
        logger_.info("[Planner] ", state_name(),
                     "  TTC=", ttc, " s",
                     "  coverage=", det.coverage_ratio,
                     "  conf=", depth.confidence);
    }

    // ── Build setpoint ───────────────────────────────────────────────────

    switch (state_) {
        case AvoidanceState::CLEAR:
            return build_forward(obs.ekf, cfg_.cruise_speed_m_s);

        case AvoidanceState::MONITORING: {
            // Scale cruise speed proportionally: slower as TTC decreases
            float t = (ttc - cfg_.ttc_brake_s) /
                      (cfg_.ttc_warn_s - cfg_.ttc_brake_s);
            t = std::max(0.1f, std::min(1.0f, t));
            return build_forward(obs.ekf, cfg_.cruise_speed_m_s * t);
        }

        case AvoidanceState::BRAKING:
            return build_hold(obs.ekf);

        case AvoidanceState::DODGING_LEFT:
        case AvoidanceState::DODGING_RIGHT: {
            VelocityCommand cmd = build_lateral_dodge(obs.ekf,
                                                      cfg_.dodge_speed_m_s,
                                                      dodge_left_);
            // Lateral offset PID correction: nudge towards the cleaner side
            // based on how centred the obstacle is in the frame.
            // error > 0 means obstacle is to the right  → push more left
            // error < 0 means obstacle is to the left   → push more right
            float dt = 1.0f / cfg_.loop_rate_hz;
            float error = det.centre_x_norm - 0.5f;   // [-0.5, +0.5]

            lateral_integral_   += error * dt;
            float lateral_deriv  = (error - lateral_prev_error_) / dt;
            lateral_prev_error_  = error;

            float pid_out = cfg_.lateral_kp * error
                          + cfg_.lateral_ki * lateral_integral_
                          + cfg_.lateral_kd * lateral_deriv;

            // pid_out is a lateral correction in body frame (positive = rightward)
            // We want to push away from the obstacle, so negate:
            float correction_body = -pid_out * cfg_.max_lateral_speed;
            float north_corr, east_corr, dummy;
            rotation_.body_to_ned(0.0f, correction_body, 0.0f,
                                   north_corr, east_corr, dummy);

            cmd.north_m_s += north_corr;
            cmd.east_m_s  += east_corr;

            // Clamp to max lateral speed
            float lateral_mag = std::sqrt(cmd.north_m_s * cmd.north_m_s +
                                          cmd.east_m_s  * cmd.east_m_s);
            if (lateral_mag > cfg_.max_lateral_speed) {
                float scale = cfg_.max_lateral_speed / lateral_mag;
                cmd.north_m_s *= scale;
                cmd.east_m_s  *= scale;
            }
            return cmd;
        }

        case AvoidanceState::CLIMBING:
            return build_climb(obs.ekf);
    }

    return build_hold(obs.ekf);
}

// ── Setpoint builders ─────────────────────────────────────────────────────────

VelocityCommand AvoidancePlannerService::build_forward(const EKFSnapshot& ekf,
                                                        float speed) {
    VelocityCommand cmd{};
    cmd.timestamp_us = now_us();
    cmd.yaw_deg      = ekf.yaw_deg;
    // Forward in body frame → rotate to NED using full R matrix
    float down_dummy;
    rotation_.body_to_ned(speed, 0.0f, 0.0f,
                           cmd.north_m_s, cmd.east_m_s, down_dummy);
    cmd.down_m_s = 0.0f;
    return cmd;
}

VelocityCommand AvoidancePlannerService::build_lateral_dodge(const EKFSnapshot& ekf,
                                                              float speed, bool left) {
    VelocityCommand cmd{};
    cmd.timestamp_us = now_us();
    cmd.yaw_deg      = ekf.yaw_deg;
    float right_speed = left ? -speed : speed;
    float down_dummy;
    rotation_.body_to_ned(0.0f, right_speed, 0.0f,
                           cmd.north_m_s, cmd.east_m_s, down_dummy);
    cmd.down_m_s = 0.0f;
    return cmd;
}

VelocityCommand AvoidancePlannerService::build_hold(const EKFSnapshot& ekf) {
    VelocityCommand cmd{};
    cmd.timestamp_us = now_us();
    cmd.yaw_deg      = ekf.yaw_deg;
    return cmd;
}

VelocityCommand AvoidancePlannerService::build_climb(const EKFSnapshot& ekf) {
    VelocityCommand cmd{};
    cmd.timestamp_us = now_us();
    cmd.yaw_deg      = ekf.yaw_deg;
    cmd.down_m_s     = -cfg_.ascent_speed_m_s;   // negative = up in NED
    return cmd;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

bool AvoidancePlannerService::select_dodge_direction(
        const std::array<float, 8>& histogram) const {
    // Sum edge density in left half (sectors 0-3) vs right half (sectors 4-7).
    // Dodge towards the half with LESS edge density (clearer path).
    float left_density  = 0.0f, right_density = 0.0f;
    for (int i = 0; i <= cfg_.left_sector_end;   i++) left_density  += histogram[static_cast<size_t>(i)];
    for (int i = cfg_.right_sector_start; i < 8; i++) right_density += histogram[static_cast<size_t>(i)];

    return (left_density <= right_density);  // true = dodge left
}

const char* AvoidancePlannerService::state_name() const {
    switch (state_) {
        case AvoidanceState::CLEAR:         return "CLEAR";
        case AvoidanceState::MONITORING:    return "MONITORING";
        case AvoidanceState::BRAKING:       return "BRAKING";
        case AvoidanceState::DODGING_LEFT:  return "DODGING_LEFT";
        case AvoidanceState::DODGING_RIGHT: return "DODGING_RIGHT";
        case AvoidanceState::CLIMBING:      return "CLIMBING";
    }
    return "UNKNOWN";
}
