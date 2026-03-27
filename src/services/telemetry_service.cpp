#include "telemetry_service.hpp"

#include <thread>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

TelemetryService::TelemetryService(MavsdkConnection&          conn,
                                    const TelemetryConfig&     cfg,
                                    DataChannel<EKFSnapshot>&  ch_ekf,
                                    EKFRingBuffer&             ekf_ring,
                                    Logger&                    logger)
    : conn_(conn)
    , cfg_(cfg)
    , ch_ekf_(ch_ekf)
    , ekf_ring_(ekf_ring)
    , logger_(logger)
{}

void TelemetryService::run() {
    logger_.info("[TelemetryService] Started");

    auto system = conn_.system();
    telemetry_   = std::make_unique<mavsdk::Telemetry>(*system);
    passthrough_ = std::make_unique<mavsdk::MavlinkPassthrough>(*system);
    debugger_    = std::make_unique<MavsdkDebugger>(*passthrough_, logger_);

    // Set telemetry rates per stream type
    telemetry_->set_rate_attitude_quaternion(cfg_.attitude_rate_hz);
    telemetry_->set_rate_attitude_euler(cfg_.attitude_rate_hz);
    telemetry_->set_rate_position_velocity_ned(cfg_.velocity_rate_hz);
    telemetry_->set_rate_position(cfg_.velocity_rate_hz);
    telemetry_->set_rate_in_air(cfg_.status_rate_hz);

    setup_subscriptions();

    if (cfg_.enable_debug_sniffer) {
        debugger_->enable_sniffer(/*watch_all=*/true);
        debugger_->enable_command_tracker();
    }
    if (cfg_.enable_statustext) {
        debugger_->enable_statustext_mirror();
    }

    logger_.info("[TelemetryService] Telemetry subscriptions active");

    // Watchdog loop: check heartbeat + periodically dump debug stats.
    auto last_dump = std::chrono::steady_clock::now();

    while (running_) {
        std::this_thread::sleep_for(500ms);

        auto now     = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_dump).count();

        if (elapsed >= cfg_.debug_dump_interval_s) {
            debugger_->dump_stats();
            last_dump = now;
        }

        if (debugger_->heartbeat_timed_out()) {
            logger_.warn("[TelemetryService] Heartbeat timeout – link may be lost");
        }
    }

    logger_.info("[TelemetryService] Stopped");
}

void TelemetryService::setup_subscriptions() {
    // Attitude quaternion
    telemetry_->subscribe_attitude_quaternion(
        [this](mavsdk::Telemetry::Quaternion q) {
            std::lock_guard<std::mutex> lk(mtx_);
            snap_.qw = q.w;
            snap_.qx = q.x;
            snap_.qy = q.y;
            snap_.qz = q.z;
            push_snapshot();
        });

    // Attitude Euler (for convenience fields and legacy consumers)
    telemetry_->subscribe_attitude_euler(
        [this](mavsdk::Telemetry::EulerAngle att) {
            std::lock_guard<std::mutex> lk(mtx_);
            snap_.roll_deg  = att.roll_deg;
            snap_.pitch_deg = att.pitch_deg;
            snap_.yaw_deg   = att.yaw_deg;
        });

    // Position + velocity in NED
    telemetry_->subscribe_position_velocity_ned(
        [this](mavsdk::Telemetry::PositionVelocityNed pv) {
            std::lock_guard<std::mutex> lk(mtx_);
            snap_.vel_north = pv.velocity.north_m_s;
            snap_.vel_east  = pv.velocity.east_m_s;
            snap_.vel_down  = pv.velocity.down_m_s;
        });

    // Relative altitude
    telemetry_->subscribe_position(
        [this](mavsdk::Telemetry::Position pos) {
            std::lock_guard<std::mutex> lk(mtx_);
            snap_.alt_rel_m = pos.relative_altitude_m;
        });

    // Armed flag
    telemetry_->subscribe_armed(
        [this](bool armed) {
            std::lock_guard<std::mutex> lk(mtx_);
            snap_.armed = armed;
            logger_.info("[TelemetryService] Armed: ", armed ? "YES" : "NO");
        });

    // In-air flag
    telemetry_->subscribe_in_air(
        [this](bool in_air) {
            std::lock_guard<std::mutex> lk(mtx_);
            snap_.in_air = in_air;
        });
}

// Called under mtx_ lock – publishes a copy of snap_ to both sinks.
void TelemetryService::push_snapshot() {
    snap_.timestamp_us = now_us();
    ch_ekf_.publish(snap_);
    ekf_ring_.push(snap_);
}
