#pragma once

#include <cstdint>
#include <array>
#include <chrono>
#include <opencv2/core.hpp>

// ── Timestamp helper ──────────────────────────────────────────────────────────

inline uint64_t now_us() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

// ── Camera ────────────────────────────────────────────────────────────────────

struct StampedFrame {
    cv::Mat  frame;
    uint64_t timestamp_us{0};
};

// ── Detection ─────────────────────────────────────────────────────────────────

struct DetectionResult {
    enum class Sector { NONE, LEFT, CENTER, RIGHT };
    enum class Threat  { NONE, FAR, MEDIUM, CLOSE, CRITICAL };

    bool     obstacle_detected{false};
    cv::Rect bounding_box{};
    float    coverage_ratio{0.0f};  // bbox_area / frame_area  (0..1)
    float    centre_x_norm{0.5f};   // normalised bbox centre X (0=left, 1=right)
    Sector   sector{Sector::NONE};

    Threat threat() const {
        if (!obstacle_detected)     return Threat::NONE;
        if (coverage_ratio < 0.03f) return Threat::FAR;
        if (coverage_ratio < 0.10f) return Threat::MEDIUM;
        if (coverage_ratio < 0.25f) return Threat::CLOSE;
        return Threat::CRITICAL;
    }
};

// DetectionResult stamped with a capture timestamp and per-sector edge density
// (8 horizontal sectors, used by DepthService for the spatial histogram).
struct StampedDetection {
    DetectionResult      result;
    std::array<float, 8> sector_edge_density{};
    uint64_t             timestamp_us{0};
};

// ── Depth / TTC ───────────────────────────────────────────────────────────────

struct DepthEstimate {
    float                ttc_s{0.0f};               // time-to-collision (s); 0 = unknown
    float                approach_speed_norm{0.0f}; // normalised rate of approach [0..1]
    float                confidence{0.0f};           // R² of linear looming fit [0..1]
    std::array<float, 8> sector_histogram{};         // edge density per sector
    uint64_t             timestamp_us{0};
};

// ── EKF Snapshot ─────────────────────────────────────────────────────────────

struct EKFSnapshot {
    // Attitude quaternion (body→NED, from EKF3 ATTITUDE_QUATERNION msg)
    float qw{1.0f}, qx{0.0f}, qy{0.0f}, qz{0.0f};

    // Velocity in NED frame (m/s)
    float vel_north{0.0f}, vel_east{0.0f}, vel_down{0.0f};

    // Euler angles (deg) – derived from quaternion for convenience
    float roll_deg{0.0f}, pitch_deg{0.0f}, yaw_deg{0.0f};

    float alt_rel_m{0.0f};  // relative altitude (m)

    // System status
    bool armed{false};
    bool in_air{false};

    uint64_t timestamp_us{0};
};

// ── Synced Observation ────────────────────────────────────────────────────────

// Output of SyncService: camera detection + depth estimate paired with the
// EKF state that was active when the frame was captured (slerp-interpolated).
struct SyncedObservation {
    StampedDetection detection;
    DepthEstimate    depth;
    EKFSnapshot      ekf;            // interpolated to detection.timestamp_us
    uint64_t         timestamp_us{0};
};

// ── Velocity Command ──────────────────────────────────────────────────────────

struct VelocityCommand {
    float    north_m_s{0.0f};
    float    east_m_s{0.0f};
    float    down_m_s{0.0f};
    float    yaw_deg{0.0f};
    uint64_t timestamp_us{0};
};

// ── Flight Commands ───────────────────────────────────────────────────────────

enum class FlightCommand {
    ARM,
    DISARM,
    TAKEOFF,
    LAND,
    HOLD,
    RTL,
    START_OFFBOARD,
    STOP_OFFBOARD,
    START_AVOIDANCE,
    STOP_AVOIDANCE,
    SHUTDOWN,
};
