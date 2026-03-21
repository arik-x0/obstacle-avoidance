#pragma once

#include <mutex>
#include <atomic>
#include <chrono>
#include <opencv2/core.hpp>

// ──────────────────────────────────────────────
// Data structures
// ──────────────────────────────────────────────

enum class FlightMode {
    Unknown,
    Ready,
    Takeoff,
    Hold,
    Mission,
    ReturnToLaunch,
    Land,
    Offboard,
    FollowMe,
    Manual,
    Altctl,
    Posctl,
    Acro,
    Stabilized,
    Rattitude
};

struct DroneState {
    // Position (GPS)
    double latitude_deg  = 0.0;
    double longitude_deg = 0.0;
    double altitude_m    = 0.0;      // AMSL
    float  rel_alt_m     = 0.0f;     // relative to home

    // Position NED
    float pos_north_m = 0.0f;
    float pos_east_m  = 0.0f;
    float pos_down_m  = 0.0f;

    // Attitude (Euler, degrees)
    float roll_deg  = 0.0f;
    float pitch_deg = 0.0f;
    float yaw_deg   = 0.0f;

    // Velocity NED (m/s)
    float vel_north_m_s = 0.0f;
    float vel_east_m_s  = 0.0f;
    float vel_down_m_s  = 0.0f;

    // System status
    bool       armed       = false;
    bool       in_air      = false;
    FlightMode flight_mode = FlightMode::Unknown;
    bool       connected   = false;

    std::chrono::steady_clock::time_point last_update{};
};

struct DetectionResult {
    enum class Sector { NONE, LEFT, CENTER, RIGHT };

    bool      obstacle_detected = false;
    cv::Rect  bounding_box;             // pixel bounding box
    float     coverage_ratio = 0.0f;   // bbox_area / frame_area (0..1)
    float     centre_x_norm  = 0.5f;   // bbox centre X normalised (0=left, 1=right)
    Sector    sector          = Sector::NONE;

    // Threat level derived from coverage_ratio
    enum class Threat { NONE, FAR, MEDIUM, CLOSE, CRITICAL };
    Threat threat() const {
        if (!obstacle_detected)        return Threat::NONE;
        if (coverage_ratio < 0.03f)    return Threat::FAR;
        if (coverage_ratio < 0.10f)    return Threat::MEDIUM;
        if (coverage_ratio < 0.25f)    return Threat::CLOSE;
        return Threat::CRITICAL;
    }

    std::chrono::steady_clock::time_point timestamp{};
};

struct VelocitySetpoint {
    float north_m_s = 0.0f;
    float east_m_s  = 0.0f;
    float down_m_s  = 0.0f;
    float yaw_deg   = 0.0f;
};

// ──────────────────────────────────────────────
// Thread-safe shared state container
// ──────────────────────────────────────────────

class SharedState {
public:
    // ── DroneState ─────────────────────────────
    void set_drone_state(const DroneState& s) {
        std::lock_guard<std::mutex> lk(drone_mtx_);
        drone_ = s;
    }
    DroneState get_drone_state() const {
        std::lock_guard<std::mutex> lk(drone_mtx_);
        return drone_;
    }

    // ── DetectionResult ────────────────────────
    void set_detection(const DetectionResult& d) {
        std::lock_guard<std::mutex> lk(det_mtx_);
        detection_ = d;
    }
    DetectionResult get_detection() const {
        std::lock_guard<std::mutex> lk(det_mtx_);
        return detection_;
    }

    // ── VelocitySetpoint ───────────────────────
    void set_setpoint(const VelocitySetpoint& v) {
        std::lock_guard<std::mutex> lk(sp_mtx_);
        setpoint_ = v;
    }
    VelocitySetpoint get_setpoint() const {
        std::lock_guard<std::mutex> lk(sp_mtx_);
        return setpoint_;
    }

    // ── Control flags ──────────────────────────
    std::atomic<bool> shutdown_requested{false};
    std::atomic<bool> land_requested{false};

private:
    mutable std::mutex drone_mtx_;
    mutable std::mutex det_mtx_;
    mutable std::mutex sp_mtx_;

    DroneState       drone_;
    DetectionResult  detection_;
    VelocitySetpoint setpoint_;
};
