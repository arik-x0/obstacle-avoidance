#pragma once

#include <memory>
#include <atomic>
#include <chrono>

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include "../core/service_base.hpp"
#include "../core/command_queue.hpp"
#include "../core/mavsdk_connection.hpp"
#include "../core/messages.hpp"
#include "../utils/logger.hpp"

struct CommandConfig {
    float  takeoff_altitude_m    = 5.0f;
    double offboard_heartbeat_ms = 400.0;  // send zero-setpoint if queue silent
};

// ── CommandService ────────────────────────────────────────────────────────────
//
// The only service that calls MAVSDK Action/Offboard APIs.
//
// Inputs (two separate queues, checked each tick):
//   cq_flight   – FlightCommand (ARM, TAKEOFF, LAND, …) – processed first (priority)
//   cq_velocity – VelocityCommand from PIDService
//
// Behaviour:
//   Priority loop: always drain FlightCommand queue before looking at velocity.
//   If neither queue has data and offboard mode is active, send a zero/hold
//   setpoint as a keepalive (MAVSDK offboard times out at 2 Hz minimum).

class CommandService : public ServiceBase {
public:
    CommandService(MavsdkConnection&              conn,
                   const CommandConfig&            cfg,
                   CommandQueue<FlightCommand>&    cq_flight,
                   CommandQueue<VelocityCommand>&  cq_velocity,
                   Logger&                         logger);

    const char* name() const override { return "CommandService"; }

    bool is_offboard_active() const { return offboard_active_.load(); }

private:
    void run() override;

    void execute_flight_command(FlightCommand cmd);
    void send_velocity(const VelocityCommand& vc);
    void send_zero_setpoint();

    MavsdkConnection&              conn_;
    CommandConfig                  cfg_;
    CommandQueue<FlightCommand>&   cq_flight_;
    CommandQueue<VelocityCommand>& cq_velocity_;
    Logger&                        logger_;

    std::unique_ptr<mavsdk::Action>   action_;
    std::unique_ptr<mavsdk::Offboard> offboard_;

    std::atomic<bool>    offboard_active_{false};
    float                current_yaw_deg_{0.0f};

    std::chrono::steady_clock::time_point last_vel_send_{};
};
