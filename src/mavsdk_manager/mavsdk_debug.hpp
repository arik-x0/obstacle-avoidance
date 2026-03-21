#pragma once

// MavsdkDebugger – deep MAVLink introspection via MavlinkPassthrough.
//
// This module is a primary demonstration of MAVSDK internal debugging.
// It hooks into MavlinkPassthrough to:
//   1. Sniff every incoming MAVLink message (ID, rate, sysid, compid)
//   2. Track COMMAND_ACK round-trip latency
//   3. Detect heartbeat timeouts
//   4. Periodically dump a statistics table to the logger
//   5. Allow message interception / injection for test scenarios
//
// NOTE: MavlinkPassthrough is marked deprecated in MAVSDK v4 in favour of
//       MavlinkDirect (JSON-based).  We use it here intentionally because:
//       - It gives direct access to mavlink_message_t structs
//       - The intercept API is uniquely suited for low-level debug work
//       - Migrating to MavlinkDirect is straightforward once the JSON
//         deserialization layer is wired up.

#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <unordered_map>
#include <vector>
#include <mutex>
#include <chrono>
#include <string>
#include <atomic>
#include <sstream>
#include <iomanip>
#include "../utils/logger.hpp"

// Well-known MAVLink common message IDs (avoid including full MAVLink headers
// here so the debug module stays dependency-light).
namespace mavlink_ids {
    constexpr uint32_t HEARTBEAT             = 0;
    constexpr uint32_t SYS_STATUS            = 1;
    constexpr uint32_t ATTITUDE              = 30;
    constexpr uint32_t ATTITUDE_QUATERNION   = 31;
    constexpr uint32_t LOCAL_POSITION_NED    = 32;
    constexpr uint32_t GLOBAL_POSITION_INT   = 33;
    constexpr uint32_t RC_CHANNELS_RAW       = 35;
    constexpr uint32_t SERVO_OUTPUT_RAW      = 36;
    constexpr uint32_t MISSION_CURRENT       = 42;
    constexpr uint32_t VFR_HUD               = 74;
    constexpr uint32_t COMMAND_ACK           = 77;
    constexpr uint32_t POSITION_TARGET_LOCAL_NED = 85;
    constexpr uint32_t HIGHRES_IMU           = 105;
    constexpr uint32_t STATUSTEXT            = 253;

    inline std::string name_of(uint32_t id) {
        switch (id) {
            case HEARTBEAT:               return "HEARTBEAT";
            case SYS_STATUS:              return "SYS_STATUS";
            case ATTITUDE:                return "ATTITUDE";
            case ATTITUDE_QUATERNION:     return "ATTITUDE_QUATERNION";
            case LOCAL_POSITION_NED:      return "LOCAL_POSITION_NED";
            case GLOBAL_POSITION_INT:     return "GLOBAL_POSITION_INT";
            case RC_CHANNELS_RAW:         return "RC_CHANNELS_RAW";
            case SERVO_OUTPUT_RAW:        return "SERVO_OUTPUT_RAW";
            case MISSION_CURRENT:         return "MISSION_CURRENT";
            case VFR_HUD:                 return "VFR_HUD";
            case COMMAND_ACK:             return "COMMAND_ACK";
            case POSITION_TARGET_LOCAL_NED: return "POSITION_TGT_NED";
            case HIGHRES_IMU:             return "HIGHRES_IMU";
            case STATUSTEXT:              return "STATUSTEXT";
            default: {
                std::ostringstream oss;
                oss << "MSG_" << id;
                return oss.str();
            }
        }
    }
}

class MavsdkDebugger {
public:
    // ── Per-message statistics ──────────────────────────────────────────────
    struct MsgStats {
        uint32_t    msg_id      = 0;
        uint64_t    count       = 0;         // total received
        uint64_t    bytes       = 0;         // total payload bytes
        double      first_ms    = 0.0;       // epoch ms at first receipt
        double      last_ms     = 0.0;       // epoch ms at last receipt
        double      min_gap_ms  = 1e9;       // minimum inter-message gap
        double      max_gap_ms  = 0.0;       // maximum inter-message gap
        double      sum_gap_ms  = 0.0;       // for avg calculation

        double avg_rate_hz() const {
            if (count < 2) return 0.0;
            double span = last_ms - first_ms;
            return span > 0.0 ? (count - 1) * 1000.0 / span : 0.0;
        }
        double avg_gap_ms() const {
            return (count > 1) ? sum_gap_ms / (count - 1) : 0.0;
        }
    };

    // ── Command ACK tracking ────────────────────────────────────────────────
    struct CmdRecord {
        uint16_t    command_id  = 0;
        double      sent_ms     = 0.0;
        double      acked_ms    = 0.0;
        uint8_t     result      = 0xFF;      // 0xFF = not acked yet
        bool        acked       = false;

        double rtt_ms() const { return acked ? acked_ms - sent_ms : -1.0; }
    };

    // ───────────────────────────────────────────────────────────────────────
    MavsdkDebugger(mavsdk::MavlinkPassthrough& passthrough, Logger& logger)
        : passthrough_(passthrough)
        , logger_(logger)
        , start_ms_(epoch_ms())
    {}

    // Install message sniffer.
    // If watch_all=true we intercept every message (slight CPU overhead).
    // Otherwise only the predefined interesting IDs are subscribed.
    void enable_sniffer(bool watch_all = true) {
        if (sniffing_.exchange(true)) return;  // already enabled

        if (watch_all) {
            // intercept_incoming_messages fires for EVERY message before it
            // is processed by MAVSDK plugins – ideal for timing analysis.
            passthrough_.intercept_incoming_messages_async(
                [this](mavlink_message_t& msg) -> bool {
                    record_message(msg);
                    return true;  // always forward
                });
            logger_.info("[MavsdkDebugger] All-message sniffer ACTIVE");
        } else {
            // Subscribe only to the interesting set to reduce overhead.
            for (uint32_t id : interesting_ids_) {
                passthrough_.subscribe_message_async(
                    id,
                    [this](const mavlink_message_t& msg) {
                        record_message(msg);
                    });
            }
            logger_.info("[MavsdkDebugger] Selective sniffer ACTIVE (",
                         interesting_ids_.size(), " message types)");
        }
    }

    // Must be called when we send a MAVLink command so we can correlate ACKs.
    void notify_command_sent(uint16_t command_id) {
        std::lock_guard<std::mutex> lk(cmds_mtx_);
        cmds_.push_back({command_id, epoch_ms(), 0.0, 0xFF, false});
        logger_.debug("[MavsdkDebugger] CMD_SEND cmd=", command_id,
                      " (", cmd_name(command_id), ')');
    }

    // Install COMMAND_ACK watcher (separate from the general sniffer).
    void enable_command_tracker() {
        if (tracking_cmds_.exchange(true)) return;

        passthrough_.subscribe_message_async(
            mavlink_ids::COMMAND_ACK,
            [this](const mavlink_message_t& msg) {
                handle_command_ack(msg);
            });
        logger_.info("[MavsdkDebugger] Command ACK tracker ACTIVE");
    }

    // Enable STATUSTEXT forwarding so firmware log lines appear in our logger.
    void enable_statustext_mirror() {
        passthrough_.subscribe_message_async(
            mavlink_ids::STATUSTEXT,
            [this](const mavlink_message_t& msg) {
                // STATUSTEXT payload: uint8 severity + char[50] text
                // Byte 0 = severity, bytes 1..50 = text (null-terminated)
                const uint8_t* p = msg.payload64 ? nullptr : nullptr;
                // Access raw payload via the mavlink_message_t union
                const char* text = reinterpret_cast<const char*>(&msg.payload64[0]) + 1;
                uint8_t severity = *reinterpret_cast<const uint8_t*>(&msg.payload64[0]);
                char buf[51] = {};
                // payload is packed – use the raw bytes
                const uint8_t* raw = reinterpret_cast<const uint8_t*>(msg.payload64);
                severity = raw[0];
                std::memcpy(buf, raw + 1, 50);
                buf[50] = '\0';

                switch (severity) {
                    case 0: case 1: case 2: case 3:
                        logger_.error("[FC] ", buf); break;
                    case 4:
                        logger_.warn("[FC] ", buf); break;
                    default:
                        logger_.info("[FC] ", buf); break;
                }
            });
        logger_.info("[MavsdkDebugger] STATUSTEXT mirror ACTIVE");
    }

    // Dump full statistics table to logger.
    void dump_stats() const {
        std::lock_guard<std::mutex> lk(stats_mtx_);

        double elapsed = epoch_ms() - start_ms_;
        std::ostringstream oss;
        oss << "\n╔══════════════════════════════════════════════════════════════╗\n"
            << "║           MAVSDK Debug Statistics  (+" << std::fixed
            << std::setprecision(1) << elapsed / 1000.0 << "s)\n"
            << "╠══════════════╦═══════╦══════════╦══════════╦══════════╦══════╣\n"
            << "║ Message      ║ Count ║  Rate Hz ║  min gap ║  max gap ║ avg gap ║\n"
            << "╠══════════════╬═══════╬══════════╬══════════╬══════════╬══════╣\n";

        for (const auto& [id, s] : msg_stats_) {
            oss << "║ " << std::left << std::setw(12) << mavlink_ids::name_of(id)
                << " ║ " << std::right << std::setw(5) << s.count
                << " ║ " << std::setw(8) << std::setprecision(1) << s.avg_rate_hz()
                << " ║ " << std::setw(6) << std::setprecision(1) << s.min_gap_ms << "ms"
                << " ║ " << std::setw(6) << std::setprecision(1) << s.max_gap_ms << "ms"
                << " ║ " << std::setw(6) << std::setprecision(1) << s.avg_gap_ms() << "ms"
                << " ║\n";
        }

        oss << "╚══════════════╩═══════╩══════════╩══════════╩══════════╩══════╝\n";

        // Command ACK summary
        std::lock_guard<std::mutex> lk2(cmds_mtx_);
        if (!cmds_.empty()) {
            oss << "\nCommand ACK history:\n";
            for (const auto& c : cmds_) {
                oss << "  CMD " << std::setw(5) << c.command_id
                    << " (" << cmd_name(c.command_id) << ") "
                    << (c.acked ? "ACK result=" + std::to_string(c.result)
                                   + " RTT=" + format_ms(c.rtt_ms())
                                 : "PENDING")
                    << '\n';
            }
        }

        logger_.debug(oss.str());
    }

    // Check if heartbeat has timed out (>3 s without one).
    bool heartbeat_timed_out() const {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        auto it = msg_stats_.find(mavlink_ids::HEARTBEAT);
        if (it == msg_stats_.end()) return true;
        return (epoch_ms() - it->second.last_ms) > 3000.0;
    }

    // Return copy of current stats (for external consumers).
    std::unordered_map<uint32_t, MsgStats> get_stats() const {
        std::lock_guard<std::mutex> lk(stats_mtx_);
        return msg_stats_;
    }

private:
    // ── Helpers ─────────────────────────────────────────────────────────────
    static double epoch_ms() {
        using namespace std::chrono;
        return duration<double, std::milli>(
                   steady_clock::now().time_since_epoch()).count();
    }

    static std::string format_ms(double ms) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << ms << "ms";
        return oss.str();
    }

    void record_message(const mavlink_message_t& msg) {
        double now = epoch_ms();
        std::lock_guard<std::mutex> lk(stats_mtx_);
        auto& s = msg_stats_[msg.msgid];
        if (s.count == 0) {
            s.msg_id   = msg.msgid;
            s.first_ms = now;
        } else {
            double gap = now - s.last_ms;
            s.min_gap_ms = std::min(s.min_gap_ms, gap);
            s.max_gap_ms = std::max(s.max_gap_ms, gap);
            s.sum_gap_ms += gap;
        }
        s.last_ms = now;
        s.count++;
        s.bytes += msg.len;
    }

    void handle_command_ack(const mavlink_message_t& msg) {
        // COMMAND_ACK layout (MAVLink common):
        //   uint16 command  (bytes 0-1)
        //   uint8  result   (byte  2)
        double now = epoch_ms();
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(msg.payload64);
        uint16_t command = static_cast<uint16_t>(raw[0]) |
                           static_cast<uint16_t>(raw[1]) << 8;
        uint8_t  result  = raw[2];

        record_message(msg);  // count it in stats too

        std::lock_guard<std::mutex> lk(cmds_mtx_);
        // Walk backwards – find the most recent unacked entry for this cmd
        for (auto it = cmds_.rbegin(); it != cmds_.rend(); ++it) {
            if (!it->acked && it->command_id == command) {
                it->acked    = true;
                it->acked_ms = now;
                it->result   = result;
                logger_.info("[MavsdkDebugger] CMD_ACK cmd=", command,
                             " (", cmd_name(command),
                             ") result=", static_cast<int>(result),
                             " RTT=", format_ms(it->rtt_ms()));
                return;
            }
        }
        // Unsolicited ACK
        logger_.warn("[MavsdkDebugger] UNSOLICITED CMD_ACK cmd=", command,
                     " result=", static_cast<int>(result));
    }

    static const char* cmd_name(uint16_t id) {
        // MAV_CMD subset relevant to this project
        switch (id) {
            case 400: return "COMPONENT_ARM_DISARM";
            case 22:  return "NAV_TAKEOFF";
            case 21:  return "NAV_LAND";
            case 20:  return "NAV_RETURN_TO_LAUNCH";
            case 176: return "DO_SET_MODE";
            case 179: return "DO_SET_HOME";
            default:  return "UNKNOWN";
        }
    }

    mavsdk::MavlinkPassthrough& passthrough_;
    Logger&                     logger_;
    double                      start_ms_;

    mutable std::mutex                         stats_mtx_;
    std::unordered_map<uint32_t, MsgStats>     msg_stats_;

    mutable std::mutex      cmds_mtx_;
    std::vector<CmdRecord>  cmds_;

    std::atomic<bool> sniffing_{false};
    std::atomic<bool> tracking_cmds_{false};

    // Message IDs that are interesting even without full sniffer
    const std::vector<uint32_t> interesting_ids_ = {
        mavlink_ids::HEARTBEAT,
        mavlink_ids::SYS_STATUS,
        mavlink_ids::ATTITUDE,
        mavlink_ids::LOCAL_POSITION_NED,
        mavlink_ids::GLOBAL_POSITION_INT,
        mavlink_ids::COMMAND_ACK,
        mavlink_ids::STATUSTEXT,
        mavlink_ids::POSITION_TARGET_LOCAL_NED,
    };
};
