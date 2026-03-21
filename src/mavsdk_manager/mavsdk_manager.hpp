#pragma once

#include <memory>
#include <string>
#include <functional>
#include <atomic>
#include <optional>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

#include "../utils/logger.hpp"
#include "../utils/shared_state.hpp"
#include "mavsdk_debug.hpp"

struct MavsdkConfig {
    std::string connection_url  = "udpin://0.0.0.0:14540";
    double  telemetry_rate_hz   = 20.0;
    double  offboard_rate_hz    = 20.0;
    double  connect_timeout_s   = 30.0;
    bool    enable_debug_sniffer = true;
    bool    enable_statustext    = true;
};

// Result type for fallible operations
enum class ManagerResult { Ok, Error, Timeout, NotConnected };

class MavsdkManager {
public:
    explicit MavsdkManager(const MavsdkConfig& cfg, SharedState& state, Logger& logger);
    ~MavsdkManager();

    // Non-copyable / non-movable (owns live MAVSDK resources)
    MavsdkManager(const MavsdkManager&) = delete;
    MavsdkManager& operator=(const MavsdkManager&) = delete;

    // ── Lifecycle ────────────────────────────────────────────────────────────
    // Block until a system with an autopilot is found or timeout elapses.
    ManagerResult connect();
    void          disconnect();
    bool          is_connected() const { return connected_; }

    // ── Flight operations ───────────────────────────────────────────────────
    ManagerResult arm();
    ManagerResult disarm();
    ManagerResult takeoff(float altitude_m = 5.0f);
    ManagerResult land();
    ManagerResult return_to_launch();
    ManagerResult hold();

    // ── Offboard control ────────────────────────────────────────────────────
    // Start offboard mode.  An initial zero-setpoint is sent first as
    // required by ArduPilot / PX4 before start() is called.
    ManagerResult start_offboard();
    ManagerResult stop_offboard();
    bool          is_offboard_active() const { return offboard_active_; }

    // Send a NED velocity + yaw setpoint.
    // MUST be called at ≥ 2 Hz while offboard mode is active.
    ManagerResult send_velocity_ned(const VelocitySetpoint& sp);

    // ── Debug access ────────────────────────────────────────────────────────
    MavsdkDebugger* debugger() { return debugger_.get(); }

    // Raw passthrough access for advanced usage / testing
    mavsdk::MavlinkPassthrough* mavlink_passthrough() {
        return mavlink_passthrough_.get();
    }

private:
    void setup_telemetry_subscriptions();

    MavsdkConfig  cfg_;
    SharedState&  state_;
    Logger&       logger_;

    mavsdk::Mavsdk mavsdk_;

    std::shared_ptr<mavsdk::System>          system_;
    std::unique_ptr<mavsdk::Telemetry>       telemetry_;
    std::unique_ptr<mavsdk::Action>          action_;
    std::unique_ptr<mavsdk::Offboard>        offboard_;
    std::unique_ptr<mavsdk::MavlinkPassthrough> mavlink_passthrough_;
    std::unique_ptr<MavsdkDebugger>          debugger_;

    std::atomic<bool> connected_{false};
    std::atomic<bool> offboard_active_{false};
};
