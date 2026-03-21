#pragma once

#include <atomic>
#include <chrono>

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/command_queue.hpp"
#include "../core/messages.hpp"
#include "../services/avoidance_planner_service.hpp"
#include "../services/pid_service.hpp"
#include "../utils/logger.hpp"

struct FlightManagerConfig {
    float  loop_rate_hz      = 5.0f;   // phase-machine poll rate
    float  takeoff_alt_m     = 5.0f;
    float  cruising_alt_m    = 5.0f;
    float  alt_reached_tol   = 0.5f;   // tolerance to consider altitude reached
    double arming_timeout_s  = 10.0;
    double takeoff_timeout_s = 30.0;
    double landing_timeout_s = 60.0;
    bool   auto_arm_takeoff  = false;  // arm + takeoff automatically on start
};

enum class FlightPhase {
    INIT,
    CONNECTED,
    ARMING,
    TAKING_OFF,
    FLYING,
    LANDING,
    LANDED,
    FAULT,
};

// ── FlightManagerService ──────────────────────────────────────────────────────
//
// Runs the top-level flight phase state machine:
//
//   INIT → CONNECTED → ARMING → TAKING_OFF → FLYING → LANDING → LANDED
//
// On each phase transition it pushes FlightCommands into cq_flight (consumed by
// CommandService) and enables/disables AvoidancePlannerService.
//
// Monitors EKFSnapshot for armed/in_air/altitude to drive transitions.
// Responds to external shutdown request (set running_ = false from SIGINT).

class FlightManagerService : public ServiceBase {
public:
    FlightManagerService(const FlightManagerConfig&     cfg,
                          DataChannel<EKFSnapshot>&       ch_ekf,
                          CommandQueue<FlightCommand>&    cq_flight,
                          AvoidancePlannerService&        planner,
                          PIDService&                     pid,
                          Logger&                         logger);

    const char* name() const override { return "FlightManagerService"; }

    FlightPhase current_phase() const { return phase_; }
    const char* phase_name()    const;

    // Request land + shutdown (called by SIGINT handler).
    void request_shutdown();

private:
    void run() override;
    void tick();

    void handle_init();
    void handle_connected();
    void handle_arming();
    void handle_taking_off();
    void handle_flying();
    void handle_landing();
    void handle_landed();

    void transition(FlightPhase next);

    FlightManagerConfig             cfg_;
    DataChannel<EKFSnapshot>&       ch_ekf_;
    CommandQueue<FlightCommand>&    cq_flight_;
    AvoidancePlannerService&        planner_;
    PIDService&                     pid_;
    Logger&                         logger_;

    FlightPhase phase_{FlightPhase::INIT};

    std::chrono::steady_clock::time_point phase_entry_time_{};

    std::atomic<bool> land_requested_{false};
    std::atomic<bool> shutdown_after_land_{false};
};
