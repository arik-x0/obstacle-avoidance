#include "mavsdk_manager.hpp"

#include <future>
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

// ── Construction / destruction ───────────────────────────────────────────────

MavsdkManager::MavsdkManager(const MavsdkConfig& cfg,
                             SharedState& state,
                             Logger& logger)
    : cfg_(cfg)
    , state_(state)
    , logger_(logger)
    , mavsdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation})
{}

MavsdkManager::~MavsdkManager() {
    disconnect();
}

// ── connect() ────────────────────────────────────────────────────────────────

ManagerResult MavsdkManager::connect() {
    logger_.info("Connecting to: ", cfg_.connection_url);

    auto conn_result = mavsdk_.add_any_connection(cfg_.connection_url);
    if (conn_result != mavsdk::ConnectionResult::Success) {
        logger_.error("Connection failed: ", conn_result);
        return ManagerResult::Error;
    }

    // Wait for a system with an autopilot to be discovered.
    auto sys_promise = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto sys_future  = sys_promise.get_future();

    mavsdk_.subscribe_on_new_system([this, &sys_promise]() {
        auto systems = mavsdk_.systems();
        for (auto& s : systems) {
            if (s->has_autopilot()) {
                logger_.info("Autopilot found – sysid=", s->get_system_id());
                try { sys_promise.set_value(s); } catch (...) {}
                // Unsubscribe by installing a no-op
                mavsdk_.subscribe_on_new_system(nullptr);
                return;
            }
        }
    });

    auto timeout = std::chrono::duration<double>(cfg_.connect_timeout_s);
    if (sys_future.wait_for(timeout) != std::future_status::ready) {
        logger_.error("Timed out waiting for autopilot (", cfg_.connect_timeout_s, "s)");
        return ManagerResult::Timeout;
    }

    system_ = sys_future.get();

    // Instantiate all plugins
    telemetry_           = std::make_unique<mavsdk::Telemetry>(*system_);
    action_              = std::make_unique<mavsdk::Action>(*system_);
    offboard_            = std::make_unique<mavsdk::Offboard>(*system_);
    mavlink_passthrough_ = std::make_unique<mavsdk::MavlinkPassthrough>(*system_);

    // Set telemetry rates
    logger_.debug("Setting telemetry rates to ", cfg_.telemetry_rate_hz, " Hz");
    telemetry_->set_rate_attitude_euler(cfg_.telemetry_rate_hz);
    telemetry_->set_rate_position(cfg_.telemetry_rate_hz);
    telemetry_->set_rate_position_velocity_ned(cfg_.telemetry_rate_hz);
    telemetry_->set_rate_in_air(cfg_.telemetry_rate_hz);
    telemetry_->set_rate_flight_mode(cfg_.telemetry_rate_hz);

    setup_telemetry_subscriptions();

    // Wire up the debug module
    debugger_ = std::make_unique<MavsdkDebugger>(*mavlink_passthrough_, logger_);
    if (cfg_.enable_debug_sniffer) {
        debugger_->enable_sniffer(/*watch_all=*/true);
        debugger_->enable_command_tracker();
    }
    if (cfg_.enable_statustext) {
        debugger_->enable_statustext_mirror();
    }

    connected_ = true;
    logger_.info("MAVLink system ready");
    return ManagerResult::Ok;
}

// ── disconnect() ─────────────────────────────────────────────────────────────

void MavsdkManager::disconnect() {
    if (!connected_) return;

    if (offboard_active_) stop_offboard();

    // Dump final debug stats
    if (debugger_) debugger_->dump_stats();

    connected_ = false;
    logger_.info("Disconnected");
}

// ── Telemetry subscriptions ───────────────────────────────────────────────────

void MavsdkManager::setup_telemetry_subscriptions() {
    // Attitude (Euler)
    telemetry_->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle att) {
        auto s = state_.get_drone_state();
        s.roll_deg  = att.roll_deg;
        s.pitch_deg = att.pitch_deg;
        s.yaw_deg   = att.yaw_deg;
        s.last_update = std::chrono::steady_clock::now();
        state_.set_drone_state(s);
    });

    // Position (GPS + relative altitude)
    telemetry_->subscribe_position([this](mavsdk::Telemetry::Position pos) {
        auto s = state_.get_drone_state();
        s.latitude_deg  = pos.latitude_deg;
        s.longitude_deg = pos.longitude_deg;
        s.altitude_m    = pos.absolute_altitude_m;
        s.rel_alt_m     = pos.relative_altitude_m;
        state_.set_drone_state(s);
    });

    // Position + velocity in NED frame
    telemetry_->subscribe_position_velocity_ned(
        [this](mavsdk::Telemetry::PositionVelocityNed pv) {
            auto s = state_.get_drone_state();
            s.pos_north_m   = pv.position.north_m;
            s.pos_east_m    = pv.position.east_m;
            s.pos_down_m    = pv.position.down_m;
            s.vel_north_m_s = pv.velocity.north_m_s;
            s.vel_east_m_s  = pv.velocity.east_m_s;
            s.vel_down_m_s  = pv.velocity.down_m_s;
            state_.set_drone_state(s);
        });

    // In-air flag
    telemetry_->subscribe_in_air([this](bool in_air) {
        auto s = state_.get_drone_state();
        s.in_air = in_air;
        state_.set_drone_state(s);
    });

    // Armed flag
    telemetry_->subscribe_armed([this](bool armed) {
        auto s = state_.get_drone_state();
        s.armed = armed;
        state_.set_drone_state(s);
        logger_.info("Armed state changed: ", armed ? "ARMED" : "DISARMED");
    });

    // Flight mode
    telemetry_->subscribe_flight_mode(
        [this](mavsdk::Telemetry::FlightMode mode) {
            auto s = state_.get_drone_state();
            // Map MAVSDK FlightMode enum to our internal enum
            switch (mode) {
                case mavsdk::Telemetry::FlightMode::Offboard:
                    s.flight_mode = FlightMode::Offboard; break;
                case mavsdk::Telemetry::FlightMode::Hold:
                    s.flight_mode = FlightMode::Hold; break;
                case mavsdk::Telemetry::FlightMode::Takeoff:
                    s.flight_mode = FlightMode::Takeoff; break;
                case mavsdk::Telemetry::FlightMode::Land:
                    s.flight_mode = FlightMode::Land; break;
                case mavsdk::Telemetry::FlightMode::ReturnToLaunch:
                    s.flight_mode = FlightMode::ReturnToLaunch; break;
                case mavsdk::Telemetry::FlightMode::Mission:
                    s.flight_mode = FlightMode::Mission; break;
                default:
                    s.flight_mode = FlightMode::Unknown; break;
            }
            state_.set_drone_state(s);
        });

    // Health / connection proxy
    telemetry_->subscribe_health([this](mavsdk::Telemetry::Health health) {
        (void)health;
        auto s = state_.get_drone_state();
        s.connected = true;
        state_.set_drone_state(s);
    });
}

// ── Flight operations ────────────────────────────────────────────────────────

ManagerResult MavsdkManager::arm() {
    if (!connected_) return ManagerResult::NotConnected;
    if (debugger_) debugger_->notify_command_sent(400); // MAV_CMD_COMPONENT_ARM_DISARM

    auto result = action_->arm();
    if (result != mavsdk::Action::Result::Success) {
        logger_.error("Arm failed: ", result);
        return ManagerResult::Error;
    }
    logger_.info("Arm command sent");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::disarm() {
    if (!connected_) return ManagerResult::NotConnected;
    if (debugger_) debugger_->notify_command_sent(400);

    auto result = action_->disarm();
    if (result != mavsdk::Action::Result::Success) {
        logger_.error("Disarm failed: ", result);
        return ManagerResult::Error;
    }
    logger_.info("Disarm command sent");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::takeoff(float altitude_m) {
    if (!connected_) return ManagerResult::NotConnected;

    action_->set_takeoff_altitude(altitude_m);
    if (debugger_) debugger_->notify_command_sent(22); // MAV_CMD_NAV_TAKEOFF

    auto result = action_->takeoff();
    if (result != mavsdk::Action::Result::Success) {
        logger_.error("Takeoff failed: ", result);
        return ManagerResult::Error;
    }
    logger_.info("Takeoff command sent (target alt: ", altitude_m, " m)");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::land() {
    if (!connected_) return ManagerResult::NotConnected;
    if (debugger_) debugger_->notify_command_sent(21); // MAV_CMD_NAV_LAND

    auto result = action_->land();
    if (result != mavsdk::Action::Result::Success) {
        logger_.error("Land failed: ", result);
        return ManagerResult::Error;
    }
    logger_.info("Land command sent");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::return_to_launch() {
    if (!connected_) return ManagerResult::NotConnected;
    if (debugger_) debugger_->notify_command_sent(20); // MAV_CMD_NAV_RETURN_TO_LAUNCH

    auto result = action_->return_to_launch();
    if (result != mavsdk::Action::Result::Success) {
        logger_.error("RTL failed: ", result);
        return ManagerResult::Error;
    }
    logger_.info("RTL command sent");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::hold() {
    if (!connected_) return ManagerResult::NotConnected;

    auto result = action_->hold();
    if (result != mavsdk::Action::Result::Success) {
        logger_.error("Hold failed: ", result);
        return ManagerResult::Error;
    }
    logger_.info("Hold command sent");
    return ManagerResult::Ok;
}

// ── Offboard control ─────────────────────────────────────────────────────────

ManagerResult MavsdkManager::start_offboard() {
    if (!connected_) return ManagerResult::NotConnected;

    // ArduPilot/PX4 require at least one setpoint before start()
    mavsdk::Offboard::VelocityNedYaw zero{};
    zero.north_m_s = 0.0f;
    zero.east_m_s  = 0.0f;
    zero.down_m_s  = 0.0f;
    zero.yaw_deg   = state_.get_drone_state().yaw_deg;
    offboard_->set_velocity_ned(zero);

    auto result = offboard_->start();
    if (result != mavsdk::Offboard::Result::Success) {
        logger_.error("Offboard start failed: ", result);
        return ManagerResult::Error;
    }
    offboard_active_ = true;
    logger_.info("Offboard mode STARTED");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::stop_offboard() {
    if (!offboard_active_) return ManagerResult::Ok;

    auto result = offboard_->stop();
    if (result != mavsdk::Offboard::Result::Success) {
        logger_.warn("Offboard stop returned: ", result);
    }
    offboard_active_ = false;
    logger_.info("Offboard mode STOPPED");
    return ManagerResult::Ok;
}

ManagerResult MavsdkManager::send_velocity_ned(const VelocitySetpoint& sp) {
    if (!connected_)       return ManagerResult::NotConnected;
    if (!offboard_active_) return ManagerResult::Error;

    mavsdk::Offboard::VelocityNedYaw cmd{};
    cmd.north_m_s = sp.north_m_s;
    cmd.east_m_s  = sp.east_m_s;
    cmd.down_m_s  = sp.down_m_s;
    cmd.yaw_deg   = sp.yaw_deg;

    auto result = offboard_->set_velocity_ned(cmd);
    if (result != mavsdk::Offboard::Result::Success) {
        logger_.warn("Velocity setpoint rejected: ", result);
        return ManagerResult::Error;
    }
    return ManagerResult::Ok;
}
