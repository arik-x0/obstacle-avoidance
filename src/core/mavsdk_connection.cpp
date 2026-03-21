#include "mavsdk_connection.hpp"

#include <future>
#include <chrono>

MavsdkConnection::MavsdkConnection()
    : mavsdk_(mavsdk::Mavsdk::Configuration{mavsdk::ComponentType::GroundStation})
{}

bool MavsdkConnection::connect(const std::string& url, double timeout_s, Logger& logger) {
    logger.info("[MavsdkConnection] Connecting to: ", url);

    auto conn_result = mavsdk_.add_any_connection(url);
    if (conn_result != mavsdk::ConnectionResult::Success) {
        logger.error("[MavsdkConnection] Connection failed: ", conn_result);
        return false;
    }

    // Wait for a system with an autopilot to be discovered.
    auto sys_promise = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto sys_future  = sys_promise.get_future();

    mavsdk_.subscribe_on_new_system([this, &sys_promise, &logger]() {
        auto systems = mavsdk_.systems();
        for (auto& s : systems) {
            if (s->has_autopilot()) {
                logger.info("[MavsdkConnection] Autopilot found – sysid=",
                            s->get_system_id());
                try { sys_promise.set_value(s); } catch (...) {}
                mavsdk_.subscribe_on_new_system(nullptr);
                return;
            }
        }
    });

    auto timeout = std::chrono::duration<double>(timeout_s);
    if (sys_future.wait_for(timeout) != std::future_status::ready) {
        logger.error("[MavsdkConnection] Timed out waiting for autopilot (",
                     timeout_s, " s)");
        return false;
    }

    system_    = sys_future.get();
    connected_ = true;
    logger.info("[MavsdkConnection] System ready");
    return true;
}
