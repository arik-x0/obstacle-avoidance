#pragma once

#include <memory>
#include <atomic>
#include <chrono>

#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/ekf_ring_buffer.hpp"
#include "../core/mavsdk_connection.hpp"
#include "../core/messages.hpp"
#include "../mavsdk_manager/mavsdk_debug.hpp"
#include "../utils/logger.hpp"

struct TelemetryConfig {
    double attitude_rate_hz      = 100.0;  // quaternion + euler (EKFRingBuffer feed)
    double velocity_rate_hz      = 50.0;   // position_velocity_ned + position (PID feed)
    double status_rate_hz        = 10.0;   // armed, in_air (state flags)
    bool   enable_debug_sniffer  = true;
    bool   enable_statustext     = true;
    double heartbeat_timeout_s   = 3.0;
    double debug_dump_interval_s = 5.0;
};

// ── TelemetryService ──────────────────────────────────────────────────────────
//
// Subscribes to MAVSDK telemetry and publishes EKFSnapshot to two sinks:
//   ch_ekf      – DataChannel<EKFSnapshot> (latest value for all consumers)
//   ekf_ring    – EKFRingBuffer (history for SyncService interpolation)
//
// Subscribes to:
//   attitude_quaternion   – qw/qx/qy/qz (EKF3 attitude, kept intact)
//   attitude_euler        – roll/pitch/yaw in degrees
//   position_velocity_ned – position + velocity in NED
//   position              – relative altitude
//   armed / in_air        – system status flags
//
// Also owns MavlinkPassthrough + MavsdkDebugger for deep MAVLink introspection.
// The service thread is a lightweight watchdog (heartbeat check + stats dump).

class TelemetryService : public ServiceBase {
public:
    TelemetryService(MavsdkConnection&          conn,
                     const TelemetryConfig&     cfg,
                     DataChannel<EKFSnapshot>&  ch_ekf,
                     EKFRingBuffer&             ekf_ring,
                     Logger&                    logger);

    const char* name() const override { return "TelemetryService"; }

    MavsdkDebugger* debugger() { return debugger_.get(); }

private:
    void run() override;
    void setup_subscriptions();
    void push_snapshot();

    MavsdkConnection&          conn_;
    TelemetryConfig            cfg_;
    DataChannel<EKFSnapshot>&  ch_ekf_;
    EKFRingBuffer&             ekf_ring_;
    Logger&                    logger_;

    std::unique_ptr<mavsdk::Telemetry>            telemetry_;
    std::unique_ptr<mavsdk::MavlinkPassthrough>   passthrough_;
    std::unique_ptr<MavsdkDebugger>               debugger_;

    // Partial snapshot assembled from multiple telemetry callbacks.
    // Protected by snap_mtx_.
    mutable std::mutex mtx_;
    EKFSnapshot        snap_{};
};
