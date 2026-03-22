#include <iostream>
#include <fstream>
#include <csignal>
#include <vector>

#include <nlohmann/json.hpp>

#include "utils/logger.hpp"
#include "core/messages.hpp"
#include "core/channel.hpp"
#include "core/command_queue.hpp"
#include "core/ekf_ring_buffer.hpp"
#include "core/mavsdk_connection.hpp"
#include "services/camera_service.hpp"
#include "services/detection_service.hpp"
#include "services/depth_service.hpp"
#include "services/telemetry_service.hpp"
#include "services/sync_service.hpp"
#include "services/avoidance_planner_service.hpp"
#include "services/pid_service.hpp"
#include "services/command_service.hpp"
#include "services/flight_manager_service.hpp"

using json = nlohmann::json;

// ── Signal handling ───────────────────────────────────────────────────────────

static FlightManagerService* g_flight_manager = nullptr;

static void signal_handler(int /*sig*/) {
    if (g_flight_manager) g_flight_manager->request_shutdown();
}

// ── Config helpers ────────────────────────────────────────────────────────────

static json load_config(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) {
        std::cerr << "Config not found: " << path << " – using defaults\n";
        return json{};
    }
    return json::parse(f, nullptr, /*exceptions=*/false);
}

static LogLevel parse_log_level(const std::string& s) {
    if (s == "DEBUG") return LogLevel::DEBUG;
    if (s == "INFO")  return LogLevel::INFO;
    if (s == "WARN")  return LogLevel::WARN;
    if (s == "ERROR") return LogLevel::ERROR;
    return LogLevel::DEBUG;
}

template<typename T>
static T get_or(const json& j, const std::string& key, T def) {
    if (j.contains(key) && !j[key].is_null()) return j[key].get<T>();
    return def;
}

// ── main ──────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::string config_path = "config/params.json";
    if (argc >= 2) config_path = argv[1];

    auto cfg = load_config(config_path);

    // ── Logger ─────────────────────────────────────────────────────────────
    LogLevel log_level = parse_log_level(
        get_or<std::string>(cfg, "log_level", "DEBUG"));

    Logger root_logger("Main", log_level);
    {
        std::string log_file = get_or<std::string>(cfg, "log_file", "");
        if (!log_file.empty()) root_logger.set_file(log_file);
    }
    root_logger.info("=== Obstacle Avoidance System (micro-service) ===");

    auto make_logger = [&](const std::string& tag) {
        return Logger(tag, log_level);
    };

    // ── Channels & queues ──────────────────────────────────────────────────

    DataChannel<StampedFrame>       ch_frame;
    DataChannel<StampedDetection>   ch_detection;
    DataChannel<DepthEstimate>      ch_depth;
    DataChannel<EKFSnapshot>        ch_ekf;
    DataChannel<SyncedObservation>  ch_synced;
    DataChannel<VelocityCommand>    ch_desired_vel;

    CommandQueue<VelocityCommand>   cq_corrected_vel{32};
    CommandQueue<FlightCommand>     cq_flight_cmd{16};

    EKFRingBuffer ekf_ring;

    // ── Service configs from JSON ──────────────────────────────────────────

    // Camera
    CameraConfig cam_cfg;
    if (cfg.contains("camera")) {
        auto& c = cfg["camera"];
        cam_cfg.device_index    = get_or<int>(c, "device_index",    cam_cfg.device_index);
        cam_cfg.source_url      = get_or<std::string>(c, "source_url", cam_cfg.source_url);
        cam_cfg.width           = get_or<int>(c, "width",           cam_cfg.width);
        cam_cfg.height          = get_or<int>(c, "height",          cam_cfg.height);
        cam_cfg.fps             = get_or<int>(c, "fps",             cam_cfg.fps);
        cam_cfg.flip_horizontal = get_or<bool>(c, "flip_horizontal",cam_cfg.flip_horizontal);
        cam_cfg.show_preview    = get_or<bool>(c, "show_preview",   cam_cfg.show_preview);
    }

    // Detector
    DetectorConfig det_cfg;
    if (cfg.contains("detector")) {
        auto& j = cfg["detector"];
        det_cfg.use_background_sub  = get_or<bool>(j,   "use_background_sub",  det_cfg.use_background_sub);
        det_cfg.canny_low           = get_or<int>(j,    "canny_low",           det_cfg.canny_low);
        det_cfg.canny_high          = get_or<int>(j,    "canny_high",          det_cfg.canny_high);
        det_cfg.blur_kernel         = get_or<int>(j,    "blur_kernel",         det_cfg.blur_kernel);
        det_cfg.min_area_ratio      = get_or<double>(j, "min_area_ratio",      det_cfg.min_area_ratio);
        det_cfg.max_area_ratio      = get_or<double>(j, "max_area_ratio",      det_cfg.max_area_ratio);
        det_cfg.left_sector_end     = get_or<float>(j,  "left_sector_end",     det_cfg.left_sector_end);
        det_cfg.right_sector_start  = get_or<float>(j,  "right_sector_start",  det_cfg.right_sector_start);
        det_cfg.draw_debug          = get_or<bool>(j,   "draw_debug",          det_cfg.draw_debug);
    }

    // Depth
    DepthConfig dep_cfg;
    if (cfg.contains("depth")) {
        auto& d = cfg["depth"];
        dep_cfg.window_size        = get_or<int>(d,   "window_size",        dep_cfg.window_size);
        dep_cfg.ttc_max_s          = get_or<float>(d, "ttc_max_s",          dep_cfg.ttc_max_s);
        dep_cfg.min_confidence     = get_or<float>(d, "min_confidence",     dep_cfg.min_confidence);
        dep_cfg.max_approach_slope = get_or<float>(d, "max_approach_slope", dep_cfg.max_approach_slope);
    }

    // Telemetry
    TelemetryConfig telem_cfg;
    if (cfg.contains("mavsdk")) {
        auto& m = cfg["mavsdk"];
        telem_cfg.attitude_rate_hz       = get_or<double>(m, "attitude_rate_hz",       telem_cfg.attitude_rate_hz);
        telem_cfg.velocity_rate_hz       = get_or<double>(m, "velocity_rate_hz",       telem_cfg.velocity_rate_hz);
        telem_cfg.status_rate_hz         = get_or<double>(m, "status_rate_hz",         telem_cfg.status_rate_hz);
        telem_cfg.enable_debug_sniffer   = get_or<bool>(m,   "enable_debug_sniffer",   telem_cfg.enable_debug_sniffer);
        telem_cfg.enable_statustext      = get_or<bool>(m,   "enable_statustext",      telem_cfg.enable_statustext);
        telem_cfg.debug_dump_interval_s  = get_or<double>(m, "debug_dump_interval_s",  telem_cfg.debug_dump_interval_s);
    }

    // Avoidance planner
    AvoidancePlannerConfig av_cfg;
    if (cfg.contains("avoidance")) {
        auto& a = cfg["avoidance"];
        av_cfg.cruise_speed_m_s  = get_or<float>(a, "cruise_speed_m_s",  av_cfg.cruise_speed_m_s);
        av_cfg.dodge_speed_m_s   = get_or<float>(a, "dodge_speed_m_s",   av_cfg.dodge_speed_m_s);
        av_cfg.ascent_speed_m_s  = get_or<float>(a, "ascent_speed_m_s",  av_cfg.ascent_speed_m_s);
        av_cfg.ttc_clear_s       = get_or<float>(a, "ttc_clear_s",       av_cfg.ttc_clear_s);
        av_cfg.ttc_warn_s        = get_or<float>(a, "ttc_warn_s",        av_cfg.ttc_warn_s);
        av_cfg.ttc_brake_s       = get_or<float>(a, "ttc_brake_s",       av_cfg.ttc_brake_s);
        av_cfg.ttc_dodge_s       = get_or<float>(a, "ttc_dodge_s",       av_cfg.ttc_dodge_s);
        av_cfg.max_forward_speed = get_or<float>(a, "max_forward_speed", av_cfg.max_forward_speed);
        av_cfg.max_lateral_speed = get_or<float>(a, "max_lateral_speed", av_cfg.max_lateral_speed);
        av_cfg.lateral_kp        = get_or<float>(a, "lateral_kp",        av_cfg.lateral_kp);
        av_cfg.lateral_ki        = get_or<float>(a, "lateral_ki",        av_cfg.lateral_ki);
        av_cfg.lateral_kd        = get_or<float>(a, "lateral_kd",        av_cfg.lateral_kd);
    }

    // PID
    PIDConfig pid_cfg;
    if (cfg.contains("pid")) {
        auto& p = cfg["pid"];
        pid_cfg.ttc_target_s             = get_or<float>(p, "ttc_target_s",             pid_cfg.ttc_target_s);
        pid_cfg.fwd_kp                   = get_or<float>(p, "fwd_kp",                   pid_cfg.fwd_kp);
        pid_cfg.fwd_ki                   = get_or<float>(p, "fwd_ki",                   pid_cfg.fwd_ki);
        pid_cfg.fwd_kd                   = get_or<float>(p, "fwd_kd",                   pid_cfg.fwd_kd);
        pid_cfg.alt_kp                   = get_or<float>(p, "alt_kp",                   pid_cfg.alt_kp);
        pid_cfg.alt_ki                   = get_or<float>(p, "alt_ki",                   pid_cfg.alt_ki);
        pid_cfg.alt_kd                   = get_or<float>(p, "alt_kd",                   pid_cfg.alt_kd);
        pid_cfg.lat_kp                   = get_or<float>(p, "lat_kp",                   pid_cfg.lat_kp);
        pid_cfg.lat_ki                   = get_or<float>(p, "lat_ki",                   pid_cfg.lat_ki);
        pid_cfg.lat_kd                   = get_or<float>(p, "lat_kd",                   pid_cfg.lat_kd);
        pid_cfg.max_delta_speed_per_tick = get_or<float>(p, "max_delta_speed_per_tick", pid_cfg.max_delta_speed_per_tick);
    }

    // Command
    CommandConfig cmd_cfg;
    if (cfg.contains("command")) {
        auto& c = cfg["command"];
        cmd_cfg.takeoff_altitude_m   = get_or<float>(c,  "takeoff_altitude_m",   cmd_cfg.takeoff_altitude_m);
        cmd_cfg.offboard_heartbeat_ms= get_or<double>(c, "offboard_heartbeat_ms",cmd_cfg.offboard_heartbeat_ms);
    } else if (cfg.contains("control")) {
        cmd_cfg.takeoff_altitude_m = get_or<float>(cfg["control"], "takeoff_alt_m", cmd_cfg.takeoff_altitude_m);
    }

    // Flight manager
    FlightManagerConfig fm_cfg;
    if (cfg.contains("control")) {
        auto& ct = cfg["control"];
        fm_cfg.loop_rate_hz     = get_or<float>(ct, "loop_rate_hz",    fm_cfg.loop_rate_hz);
        fm_cfg.takeoff_alt_m    = get_or<float>(ct, "takeoff_alt_m",   fm_cfg.takeoff_alt_m);
        fm_cfg.cruising_alt_m   = get_or<float>(ct, "cruising_alt_m",  fm_cfg.cruising_alt_m);
        fm_cfg.alt_reached_tol  = get_or<float>(ct, "alt_reached_tol", fm_cfg.alt_reached_tol);
        fm_cfg.auto_arm_takeoff = get_or<bool>(ct,  "auto_arm_takeoff",fm_cfg.auto_arm_takeoff);
    }

    // ── MAVSDK connection ──────────────────────────────────────────────────

    std::string conn_url    = "udpin://0.0.0.0:14540";
    double      conn_timeout = 30.0;
    if (cfg.contains("mavsdk")) {
        conn_url     = get_or<std::string>(cfg["mavsdk"], "connection_url",   conn_url);
        conn_timeout = get_or<double>(cfg["mavsdk"],      "connect_timeout_s", conn_timeout);
    }

    Logger conn_logger = make_logger("Connection");
    MavsdkConnection conn;
    if (!conn.connect(conn_url, conn_timeout, conn_logger)) {
        root_logger.error("Failed to connect to flight controller – exiting");
        return 1;
    }

    // ── Construct services ─────────────────────────────────────────────────

    Logger cam_logger   = make_logger("Camera");
    Logger det_logger   = make_logger("Detect");
    Logger dep_logger   = make_logger("Depth");
    Logger telem_logger = make_logger("Telem");
    Logger sync_logger  = make_logger("Sync");
    Logger plan_logger  = make_logger("Planner");
    Logger pid_logger   = make_logger("PID");
    Logger cmd_logger   = make_logger("Command");
    Logger fm_logger    = make_logger("FlightMgr");

    CameraService           svc_camera  (cam_cfg,  ch_frame,       cam_logger);
    DetectionService        svc_detect  (det_cfg,  ch_frame,       ch_detection, det_logger);
    DepthService            svc_depth   (dep_cfg,  ch_detection,   ch_depth,     dep_logger);
    TelemetryService        svc_telem   (conn,     telem_cfg,      ch_ekf,       ekf_ring, telem_logger);
    SyncService             svc_sync    (ch_detection, ch_depth,   ekf_ring,     ch_synced, sync_logger);
    AvoidancePlannerService svc_planner (av_cfg,   ch_synced,      ch_desired_vel, plan_logger);
    PIDService              svc_pid     (pid_cfg,  ch_desired_vel, ch_ekf,       ch_depth, cq_corrected_vel, pid_logger);
    CommandService          svc_cmd     (conn,     cmd_cfg,        cq_flight_cmd, cq_corrected_vel, cmd_logger);
    FlightManagerService    svc_fm      (fm_cfg,   ch_ekf,         cq_flight_cmd, svc_planner, svc_pid, fm_logger);

    // Register SIGINT handler after FlightManagerService is constructed
    g_flight_manager = &svc_fm;
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // ── Start all services ─────────────────────────────────────────────────
    // Order: data producers before consumers, MAVSDK services after connection.

    root_logger.info("Starting services...");

    svc_telem.start();    // MAVSDK telemetry → ch_ekf, ekf_ring
    svc_camera.start();   // camera → ch_frame
    svc_detect.start();   // ch_frame → ch_detection
    svc_depth.start();    // ch_detection → ch_depth
    svc_sync.start();     // ch_detection + ch_depth + ekf_ring → ch_synced
    svc_planner.start();  // ch_synced → ch_desired_vel  (starts disabled)
    svc_pid.start();      // ch_desired_vel + ch_ekf + ch_depth → cq_corrected_vel
    svc_cmd.start();      // cq_flight_cmd + cq_corrected_vel → MAVSDK
    svc_fm.start();       // phase machine, enables planner on FLYING

    root_logger.info("All services running. Waiting for flight manager to complete.");

    // Block until FlightManagerService exits (LANDED or FAULT)
    // It calls stop() on itself, so we just join its thread by stopping.
    while (svc_fm.is_running()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ── Stop all services in reverse start order ───────────────────────────

    root_logger.info("Shutting down services...");

    svc_fm.stop();
    svc_cmd.stop();
    svc_pid.stop();
    svc_planner.stop();
    svc_sync.stop();
    svc_depth.stop();
    svc_detect.stop();
    svc_camera.stop();
    svc_telem.stop();

    root_logger.info("Shutdown complete.");
    return 0;
}
