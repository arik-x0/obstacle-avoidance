#pragma once

#include "../utils/shared_state.hpp"
#include "../utils/logger.hpp"

struct AvoidanceConfig {
    float cruise_speed_m_s  = 3.0f;  // forward speed in normal flight
    float dodge_speed_m_s   = 2.0f;  // lateral speed when dodging
    float ascent_speed_m_s  = 1.0f;  // upward speed when climbing over obstacle
    float brake_distance    = 0.30f; // coverage_ratio threshold to stop forward motion
    float dodge_distance    = 0.10f; // coverage_ratio threshold to start lateral dodge
    float clear_distance    = 0.03f; // coverage_ratio threshold to consider path clear
    float max_forward_speed = 5.0f;
    float max_lateral_speed = 3.0f;
};

enum class AvoidanceState {
    CLEAR,           // No obstacle, flying forward
    MONITORING,      // Obstacle detected but far – slow down
    BRAKING,         // Obstacle close ahead – stop forward motion
    DODGING_LEFT,    // Executing left dodge manoeuvre
    DODGING_RIGHT,   // Executing right dodge manoeuvre
    CLIMBING,        // Climbing over obstacle (fallback)
};

class AvoidanceStrategy {
public:
    explicit AvoidanceStrategy(const AvoidanceConfig& cfg, Logger& logger);

    // Compute next velocity setpoint given current telemetry and detection.
    // The returned setpoint is in the drone's NED frame.
    // heading_deg is used to rotate body-frame dodge commands to NED.
    VelocitySetpoint compute(const DroneState& drone,
                             const DetectionResult& detection);

    AvoidanceState current_state() const { return state_; }
    const char*    state_name()    const;

private:
    VelocitySetpoint build_forward(const DroneState& drone, float speed);
    VelocitySetpoint build_lateral_dodge(const DroneState& drone,
                                         float speed, bool left);
    VelocitySetpoint build_hold();
    VelocitySetpoint build_climb(const DroneState& drone);

    // Rotate body-frame (forward/right) to NED using yaw angle
    static void body_to_ned(float forward, float right, float yaw_deg,
                             float& north, float& east);

    AvoidanceConfig cfg_;
    Logger&         logger_;
    AvoidanceState  state_{AvoidanceState::CLEAR};
    AvoidanceState  prev_state_{AvoidanceState::CLEAR};

    // Dodge direction is latched once we commit to avoid oscillation
    bool dodge_dir_latched_{false};
    bool dodge_left_{true};
};
