#pragma once

#include <memory>
#include <string>
#include <atomic>

#include <mavsdk/mavsdk.h>

#include "../utils/logger.hpp"

// ── MavsdkConnection ──────────────────────────────────────────────────────────
//
// Owns the mavsdk::Mavsdk instance and the discovered system.
// Neither TelemetryService nor CommandService know about each other;
// both receive a reference to this shared connection object and use it
// to instantiate their respective plugins.
//
// Usage:
//   MavsdkConnection conn;
//   if (!conn.connect("udpin://0.0.0.0:14540", 30.0, logger)) { ... }
//   TelemetryService telem(conn, ...);
//   CommandService   cmd  (conn, ...);

class MavsdkConnection {
public:
    MavsdkConnection();

    // Block until an autopilot system is discovered or timeout_s elapses.
    // Returns true on success.
    bool connect(const std::string& url, double timeout_s, Logger& logger);

    bool                             is_connected() const { return connected_.load(); }
    mavsdk::Mavsdk&                  mavsdk()             { return mavsdk_; }
    std::shared_ptr<mavsdk::System>  system()       const { return system_; }

private:
    mavsdk::Mavsdk                  mavsdk_;
    std::shared_ptr<mavsdk::System> system_;
    std::atomic<bool>               connected_{false};
};
