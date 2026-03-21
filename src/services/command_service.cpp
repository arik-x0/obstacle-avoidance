#include "command_service.hpp"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

CommandService::CommandService(MavsdkConnection&              conn,
                                const CommandConfig&            cfg,
                                CommandQueue<FlightCommand>&    cq_flight,
                                CommandQueue<VelocityCommand>&  cq_velocity,
                                Logger&                         logger)
    : conn_(conn)
    , cfg_(cfg)
    , cq_flight_(cq_flight)
    , cq_velocity_(cq_velocity)
    , logger_(logger)
{
    last_vel_send_ = Clock::now();
}

void CommandService::run() {
    logger_.info("[CommandService] Started");

    auto system   = conn_.system();
    action_       = std::make_unique<mavsdk::Action>(*system);
    offboard_     = std::make_unique<mavsdk::Offboard>(*system);

    while (running_) {
        // ── Priority 1: flight commands ───────────────────────────────────
        FlightCommand fc;
        while (cq_flight_.try_pop(fc)) {
            execute_flight_command(fc);
        }

        // ── Priority 2: velocity commands ────────────────────────────────
        VelocityCommand vc;
        if (cq_velocity_.try_pop(vc)) {
            send_velocity(vc);
            last_vel_send_    = Clock::now();
            current_yaw_deg_  = vc.yaw_deg;
        }

        // ── Offboard keepalive ───────────────────────────────────────────
        if (offboard_active_) {
            double ms_since_send = std::chrono::duration<double, std::milli>(
                                       Clock::now() - last_vel_send_).count();
            if (ms_since_send >= cfg_.offboard_heartbeat_ms) {
                send_zero_setpoint();
                last_vel_send_ = Clock::now();
            }
        }

        std::this_thread::sleep_for(1ms);
    }

    // Cleanup: stop offboard if still active
    if (offboard_active_) {
        offboard_->stop();
        offboard_active_ = false;
    }

    logger_.info("[CommandService] Stopped");
}

// ── Flight command executor ───────────────────────────────────────────────────

void CommandService::execute_flight_command(FlightCommand cmd) {
    switch (cmd) {

        case FlightCommand::ARM: {
            auto r = action_->arm();
            if (r != mavsdk::Action::Result::Success)
                logger_.error("[CommandService] ARM failed: ", r);
            else
                logger_.info("[CommandService] ARM sent");
            break;
        }

        case FlightCommand::DISARM: {
            auto r = action_->disarm();
            if (r != mavsdk::Action::Result::Success)
                logger_.warn("[CommandService] DISARM failed: ", r);
            else
                logger_.info("[CommandService] DISARM sent");
            break;
        }

        case FlightCommand::TAKEOFF: {
            action_->set_takeoff_altitude(cfg_.takeoff_altitude_m);
            auto r = action_->takeoff();
            if (r != mavsdk::Action::Result::Success)
                logger_.error("[CommandService] TAKEOFF failed: ", r);
            else
                logger_.info("[CommandService] TAKEOFF sent (alt=",
                             cfg_.takeoff_altitude_m, " m)");
            break;
        }

        case FlightCommand::LAND: {
            // Stop offboard before landing
            if (offboard_active_) {
                offboard_->stop();
                offboard_active_ = false;
            }
            auto r = action_->land();
            if (r != mavsdk::Action::Result::Success)
                logger_.error("[CommandService] LAND failed: ", r);
            else
                logger_.info("[CommandService] LAND sent");
            break;
        }

        case FlightCommand::HOLD: {
            auto r = action_->hold();
            if (r != mavsdk::Action::Result::Success)
                logger_.warn("[CommandService] HOLD failed: ", r);
            else
                logger_.info("[CommandService] HOLD sent");
            break;
        }

        case FlightCommand::RTL: {
            auto r = action_->return_to_launch();
            if (r != mavsdk::Action::Result::Success)
                logger_.error("[CommandService] RTL failed: ", r);
            else
                logger_.info("[CommandService] RTL sent");
            break;
        }

        case FlightCommand::START_OFFBOARD: {
            // Must send at least one setpoint before calling start()
            send_zero_setpoint();
            auto r = offboard_->start();
            if (r != mavsdk::Offboard::Result::Success) {
                logger_.error("[CommandService] START_OFFBOARD failed: ", r);
            } else {
                offboard_active_ = true;
                last_vel_send_   = Clock::now();
                logger_.info("[CommandService] Offboard STARTED");
            }
            break;
        }

        case FlightCommand::STOP_OFFBOARD: {
            auto r = offboard_->stop();
            if (r != mavsdk::Offboard::Result::Success)
                logger_.warn("[CommandService] STOP_OFFBOARD: ", r);
            offboard_active_ = false;
            logger_.info("[CommandService] Offboard STOPPED");
            break;
        }

        case FlightCommand::START_AVOIDANCE:
        case FlightCommand::STOP_AVOIDANCE:
        case FlightCommand::SHUTDOWN:
            // Handled by FlightManagerService; forwarded here only for logging.
            logger_.debug("[CommandService] Ignoring non-MAVSDK command");
            break;
    }
}

// ── Velocity send ─────────────────────────────────────────────────────────────

void CommandService::send_velocity(const VelocityCommand& vc) {
    if (!offboard_active_) return;

    mavsdk::Offboard::VelocityNedYaw cmd{};
    cmd.north_m_s = vc.north_m_s;
    cmd.east_m_s  = vc.east_m_s;
    cmd.down_m_s  = vc.down_m_s;
    cmd.yaw_deg   = vc.yaw_deg;

    auto r = offboard_->set_velocity_ned(cmd);
    if (r != mavsdk::Offboard::Result::Success) {
        logger_.warn("[CommandService] Velocity rejected: ", r);
    }
}

void CommandService::send_zero_setpoint() {
    mavsdk::Offboard::VelocityNedYaw zero{};
    zero.north_m_s = 0.0f;
    zero.east_m_s  = 0.0f;
    zero.down_m_s  = 0.0f;
    zero.yaw_deg   = current_yaw_deg_;
    offboard_->set_velocity_ned(zero);
}
