#pragma once

#include <memory>
#include <chrono>

#include "../mavsdk_manager/mavsdk_manager.hpp"
#include "../camera/camera_interface.hpp"
#include "../detection/object_detector.hpp"
#include "avoidance_strategy.hpp"
#include "../utils/shared_state.hpp"
#include "../utils/logger.hpp"

struct ControlConfig {
    float loop_rate_hz     = 20.0f;  // main loop frequency
    float takeoff_alt_m    = 5.0f;   // target takeoff altitude
    float cruising_alt_m   = 5.0f;   // altitude to maintain during cruise
    float alt_reached_tol  = 0.5f;   // altitude tolerance (m) for takeoff complete
    float debug_dump_hz    = 0.2f;   // how often to dump MAVSDK stats (Hz)
    bool  enable_camera    = true;
    bool  enable_avoidance = true;
    bool  auto_arm_takeoff = false;  // if true, arm+takeoff automatically on start
};

// Top-level flight state machine
enum class FlightPhase {
    INIT,        // Waiting for MAVSDK connection
    CONNECTED,   // Connected, waiting for arming command
    ARMING,      // Arm command sent, waiting for armed status
    TAKING_OFF,  // Takeoff command sent, climbing
    FLYING,      // Cruising, avoidance active
    LANDING,     // Land command sent
    LANDED,      // On ground, disarmed
    FAULT,       // Unrecoverable error
};

class ControlLoop {
public:
    ControlLoop(MavsdkManager&    mavsdk,
                CameraInterface&  camera,
                ObjectDetector&   detector,
                AvoidanceStrategy& avoidance,
                SharedState&       state,
                const ControlConfig& cfg,
                Logger&            logger);

    // Run the loop until shutdown_requested is set in SharedState.
    void run();

    // Request a graceful stop (also set via SharedState::shutdown_requested)
    void request_shutdown();

    FlightPhase current_phase() const { return phase_; }
    const char* phase_name()    const;

private:
    // One iteration of the control loop
    void tick();

    // Per-phase handlers
    void handle_init();
    void handle_connected();
    void handle_arming();
    void handle_taking_off();
    void handle_flying();
    void handle_landing();
    void handle_landed();

    void transition(FlightPhase next);

    MavsdkManager&    mavsdk_;
    CameraInterface&  camera_;
    ObjectDetector&   detector_;
    AvoidanceStrategy& avoidance_;
    SharedState&       state_;
    ControlConfig      cfg_;
    Logger&            logger_;

    FlightPhase  phase_{FlightPhase::INIT};
    bool         shutdown_{false};

    std::chrono::steady_clock::time_point phase_entry_time_{};
    std::chrono::steady_clock::time_point last_debug_dump_{};
    std::chrono::steady_clock::time_point last_setpoint_send_{};

    // Statistics
    uint64_t tick_count_{0};
    double   loop_overrun_ms_{0.0};
};
