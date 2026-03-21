#include "avoidance_strategy.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

AvoidanceStrategy::AvoidanceStrategy(const AvoidanceConfig& cfg, Logger& logger)
    : cfg_(cfg), logger_(logger) {}

VelocitySetpoint AvoidanceStrategy::compute(const DroneState& drone,
                                             const DetectionResult& det) {
    prev_state_ = state_;
    auto threat = det.threat();

    // ── State transitions ──────────────────────────────────────────────────

    switch (state_) {
        case AvoidanceState::CLEAR:
            if (threat >= DetectionResult::Threat::MEDIUM &&
                det.sector == DetectionResult::Sector::CENTER) {
                state_ = AvoidanceState::MONITORING;
            }
            break;

        case AvoidanceState::MONITORING:
            if (threat == DetectionResult::Threat::NONE ||
                det.sector != DetectionResult::Sector::CENTER) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
            } else if (threat >= DetectionResult::Threat::CLOSE) {
                state_ = AvoidanceState::BRAKING;
            }
            break;

        case AvoidanceState::BRAKING:
            if (threat <= DetectionResult::Threat::FAR) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
            } else if (threat >= DetectionResult::Threat::CLOSE) {
                // Decide dodge direction once and latch it
                if (!dodge_dir_latched_) {
                    // Dodge away from the side where the obstacle is heavier
                    dodge_left_        = (det.centre_x_norm >= 0.5f);
                    dodge_dir_latched_ = true;
                    logger_.info("[Avoidance] Dodge direction latched: ",
                                 dodge_left_ ? "LEFT" : "RIGHT",
                                 " (obstacle centre_x=", det.centre_x_norm, ")");
                }
                state_ = dodge_left_ ? AvoidanceState::DODGING_LEFT
                                     : AvoidanceState::DODGING_RIGHT;
            }
            break;

        case AvoidanceState::DODGING_LEFT:
        case AvoidanceState::DODGING_RIGHT:
            if (threat == DetectionResult::Threat::NONE) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
                logger_.info("[Avoidance] Path clear – resuming CLEAR");
            } else if (threat == DetectionResult::Threat::CRITICAL) {
                // If dodge isn't working, climb as last resort
                state_ = AvoidanceState::CLIMBING;
                logger_.warn("[Avoidance] Dodge failed – climbing over obstacle");
            }
            break;

        case AvoidanceState::CLIMBING:
            if (threat <= DetectionResult::Threat::FAR) {
                state_ = AvoidanceState::CLEAR;
                dodge_dir_latched_ = false;
                logger_.info("[Avoidance] Obstacle cleared above – CLEAR");
            }
            break;
    }

    // Log state changes
    if (state_ != prev_state_) {
        logger_.info("[Avoidance] State: ", state_name(),
                     "  threat=", static_cast<int>(threat),
                     "  sector=", static_cast<int>(det.sector));
    }

    // ── Compute setpoint ───────────────────────────────────────────────────

    switch (state_) {
        case AvoidanceState::CLEAR:
            return build_forward(drone, cfg_.cruise_speed_m_s);

        case AvoidanceState::MONITORING:
            // Slow down proportionally based on coverage
            {
                float scale = 1.0f - (det.coverage_ratio / cfg_.brake_distance);
                scale = std::max(0.2f, std::min(1.0f, scale));
                return build_forward(drone, cfg_.cruise_speed_m_s * scale);
            }

        case AvoidanceState::BRAKING:
            return build_hold();

        case AvoidanceState::DODGING_LEFT:
            return build_lateral_dodge(drone, cfg_.dodge_speed_m_s, /*left=*/true);

        case AvoidanceState::DODGING_RIGHT:
            return build_lateral_dodge(drone, cfg_.dodge_speed_m_s, /*left=*/false);

        case AvoidanceState::CLIMBING:
            return build_climb(drone);
    }

    return build_hold();  // unreachable, but keeps compiler happy
}

// ── Setpoint builders ─────────────────────────────────────────────────────────

VelocitySetpoint AvoidanceStrategy::build_forward(const DroneState& drone,
                                                   float speed) {
    VelocitySetpoint sp;
    float n, e;
    body_to_ned(speed, 0.0f, drone.yaw_deg, n, e);
    sp.north_m_s = n;
    sp.east_m_s  = e;
    sp.down_m_s  = 0.0f;
    sp.yaw_deg   = drone.yaw_deg;
    return sp;
}

VelocitySetpoint AvoidanceStrategy::build_lateral_dodge(const DroneState& drone,
                                                         float speed, bool left) {
    VelocitySetpoint sp;
    float right_speed = left ? -speed : speed;
    float n, e;
    body_to_ned(0.0f, right_speed, drone.yaw_deg, n, e);
    sp.north_m_s = n;
    sp.east_m_s  = e;
    sp.down_m_s  = 0.0f;
    sp.yaw_deg   = drone.yaw_deg;
    return sp;
}

VelocitySetpoint AvoidanceStrategy::build_hold() {
    VelocitySetpoint sp{};
    return sp;
}

VelocitySetpoint AvoidanceStrategy::build_climb(const DroneState& drone) {
    VelocitySetpoint sp;
    sp.north_m_s = 0.0f;
    sp.east_m_s  = 0.0f;
    sp.down_m_s  = -cfg_.ascent_speed_m_s;  // negative = up in NED
    sp.yaw_deg   = drone.yaw_deg;
    return sp;
}

void AvoidanceStrategy::body_to_ned(float forward, float right,
                                     float yaw_deg,
                                     float& north, float& east) {
    float yaw_rad = static_cast<float>(yaw_deg * M_PI / 180.0);
    north =  forward * std::cos(yaw_rad) - right * std::sin(yaw_rad);
    east  =  forward * std::sin(yaw_rad) + right * std::cos(yaw_rad);
}

const char* AvoidanceStrategy::state_name() const {
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
