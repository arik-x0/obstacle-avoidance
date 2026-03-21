#include "flight_manager_service.hpp"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

FlightManagerService::FlightManagerService(
        const FlightManagerConfig&     cfg,
        DataChannel<EKFSnapshot>&       ch_ekf,
        CommandQueue<FlightCommand>&    cq_flight,
        AvoidancePlannerService&        planner,
        PIDService&                     pid,
        Logger&                         logger)
    : cfg_(cfg)
    , ch_ekf_(ch_ekf)
    , cq_flight_(cq_flight)
    , planner_(planner)
    , pid_(pid)
    , logger_(logger)
{
    phase_entry_time_ = Clock::now();
}

void FlightManagerService::request_shutdown() {
    land_requested_      = true;
    shutdown_after_land_ = true;
}

void FlightManagerService::run() {
    logger_.info("[FlightManagerService] Started");

    const auto period = std::chrono::duration<double>(1.0 / cfg_.loop_rate_hz);

    while (running_) {
        auto tick_start = Clock::now();

        tick();

        // Stop the service loop if we're done
        if (phase_ == FlightPhase::LANDED) {
            running_ = false;
            break;
        }

        auto elapsed   = Clock::now() - tick_start;
        auto sleep_for = period - elapsed;
        if (sleep_for.count() > 0) {
            std::this_thread::sleep_for(sleep_for);
        }
    }

    logger_.info("[FlightManagerService] Stopped (phase=", phase_name(), ")");
}

void FlightManagerService::tick() {
    // Handle external land request (e.g. SIGINT)
    if (land_requested_ && phase_ == FlightPhase::FLYING) {
        logger_.info("[FlightManagerService] Land requested");
        planner_.disable();
        cq_flight_.push(FlightCommand::STOP_OFFBOARD);
        cq_flight_.push(FlightCommand::LAND);
        transition(FlightPhase::LANDING);
        return;
    }

    switch (phase_) {
        case FlightPhase::INIT:       handle_init();       break;
        case FlightPhase::CONNECTED:  handle_connected();  break;
        case FlightPhase::ARMING:     handle_arming();     break;
        case FlightPhase::TAKING_OFF: handle_taking_off(); break;
        case FlightPhase::FLYING:     handle_flying();     break;
        case FlightPhase::LANDING:    handle_landing();    break;
        case FlightPhase::LANDED:     handle_landed();     break;
        case FlightPhase::FAULT:      break;
    }
}

// ── Phase handlers ────────────────────────────────────────────────────────────

void FlightManagerService::handle_init() {
    EKFSnapshot ekf = ch_ekf_.latest();
    if (ekf.timestamp_us == 0) return;   // no telemetry yet

    transition(FlightPhase::CONNECTED);

    if (cfg_.auto_arm_takeoff) {
        logger_.info("[FlightManagerService] auto_arm_takeoff – arming");
        cq_flight_.push(FlightCommand::ARM);
        transition(FlightPhase::ARMING);
    }
}

void FlightManagerService::handle_connected() {
    double elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_).count();
    if (static_cast<int>(elapsed) % 5 == 0 &&
        static_cast<int>(elapsed * cfg_.loop_rate_hz) %
            static_cast<int>(cfg_.loop_rate_hz) == 0) {
        logger_.info("[FlightManagerService] Waiting for arm command... (",
                     static_cast<int>(elapsed), " s)");
    }
}

void FlightManagerService::handle_arming() {
    EKFSnapshot ekf = ch_ekf_.latest();

    if (ekf.armed) {
        logger_.info("[FlightManagerService] ARMED – sending TAKEOFF");
        cq_flight_.push(FlightCommand::TAKEOFF);
        transition(FlightPhase::TAKING_OFF);
        return;
    }

    double elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_).count();
    if (elapsed > cfg_.arming_timeout_s) {
        logger_.error("[FlightManagerService] Arming timeout – FAULT");
        transition(FlightPhase::FAULT);
    }
}

void FlightManagerService::handle_taking_off() {
    EKFSnapshot ekf = ch_ekf_.latest();

    if (ekf.alt_rel_m >= cfg_.takeoff_alt_m - cfg_.alt_reached_tol) {
        logger_.info("[FlightManagerService] Altitude reached (", ekf.alt_rel_m, " m)");

        // Tell PIDService which altitude to hold
        pid_.set_target_altitude(cfg_.cruising_alt_m);

        // Switch to offboard
        cq_flight_.push(FlightCommand::START_OFFBOARD);
        cq_flight_.push(FlightCommand::START_AVOIDANCE);

        planner_.enable();
        transition(FlightPhase::FLYING);
        return;
    }

    double elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_).count();
    if (elapsed > cfg_.takeoff_timeout_s) {
        logger_.error("[FlightManagerService] Takeoff timeout – FAULT");
        transition(FlightPhase::FAULT);
    }
}

void FlightManagerService::handle_flying() {
    // Nothing to do – AvoidancePlannerService and PIDService run autonomously.
    // We just monitor for the land request handled in tick().
}

void FlightManagerService::handle_landing() {
    EKFSnapshot ekf = ch_ekf_.latest();

    if (!ekf.in_air) {
        logger_.info("[FlightManagerService] Landed");
        cq_flight_.push(FlightCommand::DISARM);
        transition(FlightPhase::LANDED);
        return;
    }

    double elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_).count();
    if (elapsed > cfg_.landing_timeout_s) {
        logger_.warn("[FlightManagerService] Landing timeout – forcing LANDED");
        transition(FlightPhase::LANDED);
    }
}

void FlightManagerService::handle_landed() {
    logger_.info("[FlightManagerService] Landed and disarmed. Shutting down.");
    running_ = false;
}

// ── Utilities ─────────────────────────────────────────────────────────────────

void FlightManagerService::transition(FlightPhase next) {
    logger_.info("[FlightManagerService] Phase: ", phase_name(), " → ", [next]() {
        switch (next) {
            case FlightPhase::INIT:       return "INIT";
            case FlightPhase::CONNECTED:  return "CONNECTED";
            case FlightPhase::ARMING:     return "ARMING";
            case FlightPhase::TAKING_OFF: return "TAKING_OFF";
            case FlightPhase::FLYING:     return "FLYING";
            case FlightPhase::LANDING:    return "LANDING";
            case FlightPhase::LANDED:     return "LANDED";
            case FlightPhase::FAULT:      return "FAULT";
        }
        return "?";
    }());
    phase_            = next;
    phase_entry_time_ = Clock::now();
}

const char* FlightManagerService::phase_name() const {
    switch (phase_) {
        case FlightPhase::INIT:       return "INIT";
        case FlightPhase::CONNECTED:  return "CONNECTED";
        case FlightPhase::ARMING:     return "ARMING";
        case FlightPhase::TAKING_OFF: return "TAKING_OFF";
        case FlightPhase::FLYING:     return "FLYING";
        case FlightPhase::LANDING:    return "LANDING";
        case FlightPhase::LANDED:     return "LANDED";
        case FlightPhase::FAULT:      return "FAULT";
    }
    return "UNKNOWN";
}
