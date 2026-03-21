#include "control_loop.hpp"

#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

ControlLoop::ControlLoop(MavsdkManager&    mavsdk,
                         CameraInterface&  camera,
                         ObjectDetector&   detector,
                         AvoidanceStrategy& avoidance,
                         SharedState&       state,
                         const ControlConfig& cfg,
                         Logger&            logger)
    : mavsdk_(mavsdk)
    , camera_(camera)
    , detector_(detector)
    , avoidance_(avoidance)
    , state_(state)
    , cfg_(cfg)
    , logger_(logger)
{
    phase_entry_time_ = Clock::now();
    last_debug_dump_  = Clock::now();
}

void ControlLoop::run() {
    const auto period = std::chrono::duration<double>(1.0 / cfg_.loop_rate_hz);
    logger_.info("Control loop starting at ", cfg_.loop_rate_hz, " Hz");

    while (!shutdown_ && !state_.shutdown_requested) {
        auto tick_start = Clock::now();

        tick();
        ++tick_count_;

        // Enforce loop rate – sleep for the remaining period
        auto elapsed = Clock::now() - tick_start;
        auto sleep_for = period - elapsed;

        if (sleep_for.count() > 0) {
            std::this_thread::sleep_for(sleep_for);
        } else {
            double overrun_ms = -std::chrono::duration<double, std::milli>(sleep_for).count();
            loop_overrun_ms_ = overrun_ms;
            if (static_cast<uint64_t>(tick_count_) % 100 == 0) {
                logger_.warn("Control loop overrun: ", overrun_ms, " ms");
            }
        }
    }

    logger_.info("Control loop stopped after ", tick_count_, " ticks");
}

void ControlLoop::request_shutdown() {
    shutdown_ = true;
}

// ── Main tick ─────────────────────────────────────────────────────────────────

void ControlLoop::tick() {
    // Periodic MAVSDK debug stats dump
    if (mavsdk_.debugger()) {
        auto now = Clock::now();
        double elapsed = std::chrono::duration<double>(now - last_debug_dump_).count();
        if (elapsed >= 1.0 / cfg_.debug_dump_hz) {
            mavsdk_.debugger()->dump_stats();
            last_debug_dump_ = now;

            // Heartbeat watchdog
            if (mavsdk_.debugger()->heartbeat_timed_out() &&
                phase_ != FlightPhase::INIT) {
                logger_.error("HEARTBEAT TIMEOUT – entering FAULT");
                transition(FlightPhase::FAULT);
                return;
            }
        }
    }

    // Handle land request from external signal (e.g. SIGINT)
    if (state_.land_requested && phase_ == FlightPhase::FLYING) {
        logger_.info("Land requested via signal");
        mavsdk_.land();
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
        case FlightPhase::FAULT:
            // Stay here; operator must restart
            break;
    }
}

// ── Phase handlers ────────────────────────────────────────────────────────────

void ControlLoop::handle_init() {
    if (mavsdk_.is_connected()) {
        transition(FlightPhase::CONNECTED);

        if (cfg_.auto_arm_takeoff) {
            logger_.info("auto_arm_takeoff enabled – arming");
            mavsdk_.arm();
            transition(FlightPhase::ARMING);
        }
    }
}

void ControlLoop::handle_connected() {
    // Waiting for arm command – nothing to do autonomously here unless
    // auto_arm_takeoff is set (handled in handle_init).
    auto elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_);
    if (static_cast<int>(elapsed.count()) % 5 == 0 &&
        static_cast<int>(elapsed.count() * cfg_.loop_rate_hz) % static_cast<int>(cfg_.loop_rate_hz) == 0) {
        logger_.info("Waiting for arm command... (", static_cast<int>(elapsed.count()), "s)");
    }
}

void ControlLoop::handle_arming() {
    auto drone = state_.get_drone_state();

    if (drone.armed) {
        logger_.info("Vehicle ARMED – sending takeoff");
        mavsdk_.takeoff(cfg_.takeoff_alt_m);
        transition(FlightPhase::TAKING_OFF);
        return;
    }

    // Timeout: if arm takes more than 10 s, transition to fault
    auto elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_);
    if (elapsed.count() > 10.0) {
        logger_.error("Arming timeout – check pre-arm checks");
        transition(FlightPhase::FAULT);
    }
}

void ControlLoop::handle_taking_off() {
    auto drone = state_.get_drone_state();

    // Check if we've reached the target altitude (within tolerance)
    if (drone.rel_alt_m >= cfg_.takeoff_alt_m - cfg_.alt_reached_tol) {
        logger_.info("Target altitude reached: ", drone.rel_alt_m, " m");

        // Start camera if configured
        if (cfg_.enable_camera && !camera_.is_running()) {
            if (!camera_.start()) {
                logger_.warn("Camera failed to start – avoidance disabled");
            }
        }

        // Switch to offboard for velocity control
        auto res = mavsdk_.start_offboard();
        if (res != ManagerResult::Ok) {
            logger_.error("Failed to start offboard – holding position");
            mavsdk_.hold();
        }

        transition(FlightPhase::FLYING);
        return;
    }

    // Timeout: if takeoff takes > 30 s, something is wrong
    auto elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_);
    if (elapsed.count() > 30.0) {
        logger_.error("Takeoff timeout");
        transition(FlightPhase::FAULT);
    }
}

void ControlLoop::handle_flying() {
    DetectionResult det;

    // ── Vision pipeline ────────────────────────────────────────────────────
    if (cfg_.enable_camera && camera_.is_running() && camera_.has_new_frame()) {
        cv::Mat frame = camera_.get_latest_frame();
        if (!frame.empty()) {
            det = detector_.detect(frame);
            state_.set_detection(det);

            if (det.obstacle_detected) {
                logger_.debug("[Flying] Obstacle detected – coverage=",
                              det.coverage_ratio,
                              " sector=", static_cast<int>(det.sector),
                              " threat=", static_cast<int>(det.threat()));
            }
        }
    } else {
        det = state_.get_detection();
    }

    // ── Avoidance ──────────────────────────────────────────────────────────
    VelocitySetpoint sp;

    if (cfg_.enable_avoidance && mavsdk_.is_offboard_active()) {
        auto drone = state_.get_drone_state();
        sp = avoidance_.compute(drone, det);
    }
    // If avoidance is disabled, sp is zero (hover)

    state_.set_setpoint(sp);

    // Send setpoint to flight controller
    // MAVSDK offboard times out if setpoints stop – must keep sending at ≥2 Hz.
    mavsdk_.send_velocity_ned(sp);
}

void ControlLoop::handle_landing() {
    auto drone = state_.get_drone_state();

    if (!drone.in_air) {
        logger_.info("Vehicle landed");
        mavsdk_.disarm();
        transition(FlightPhase::LANDED);
    }

    // Timeout
    auto elapsed = std::chrono::duration<double>(Clock::now() - phase_entry_time_);
    if (elapsed.count() > 60.0) {
        logger_.warn("Landing timeout – forcing LANDED state");
        transition(FlightPhase::LANDED);
    }
}

void ControlLoop::handle_landed() {
    // Nothing further to do – request shutdown
    logger_.info("Landed and disarmed. Requesting shutdown.");
    shutdown_ = true;
}

// ── Utilities ─────────────────────────────────────────────────────────────────

void ControlLoop::transition(FlightPhase next) {
    logger_.info("Phase: ", phase_name(), " -> [", [next]() -> const char* {
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
    }(), "]");

    phase_       = next;
    phase_entry_time_ = Clock::now();
}

const char* ControlLoop::phase_name() const {
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
