#include "camera_service.hpp"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

CameraService::CameraService(const CameraConfig&        cfg,
                              DataChannel<StampedFrame>& ch_frame,
                              Logger&                    logger)
    : camera_(cfg, logger)
    , ch_frame_(ch_frame)
    , logger_(logger)
{}

void CameraService::stop() {
    ServiceBase::stop();
    camera_.stop();
}

void CameraService::run() {
    if (!camera_.start()) {
        logger_.error("[CameraService] Failed to open camera – service exiting");
        running_ = false;
        return;
    }

    logger_.info("[CameraService] Started");

    while (running_) {
        if (camera_.has_new_frame()) {
            cv::Mat frame = camera_.get_latest_frame();
            if (!frame.empty()) {
                StampedFrame sf;
                sf.frame        = std::move(frame);
                sf.timestamp_us = now_us();
                ch_frame_.publish(std::move(sf));
            }
        } else {
            std::this_thread::sleep_for(1ms);
        }
    }

    camera_.stop();
    logger_.info("[CameraService] Stopped");
}
