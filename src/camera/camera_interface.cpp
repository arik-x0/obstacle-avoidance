#include "camera_interface.hpp"

#include <chrono>
#include <stdexcept>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std::chrono_literals;

CameraInterface::CameraInterface(const CameraConfig& cfg, Logger& logger)
    : cfg_(cfg), logger_(logger) {}

CameraInterface::~CameraInterface() {
    stop();
}

bool CameraInterface::start() {
    if (running_) return true;

    // Open the source
    if (!cfg_.source_url.empty()) {
        logger_.info("Opening camera source: ", cfg_.source_url);
        capture_.open(cfg_.source_url, cv::CAP_GSTREAMER);
        if (!capture_.isOpened()) {
            // Retry without explicit backend
            capture_.open(cfg_.source_url);
        }
    } else {
        logger_.info("Opening camera device index: ", cfg_.device_index);
        capture_.open(cfg_.device_index);
    }

    if (!capture_.isOpened()) {
        logger_.error("Failed to open camera");
        return false;
    }

    // Apply requested resolution / FPS
    if (cfg_.width > 0 && cfg_.height > 0) {
        capture_.set(cv::CAP_PROP_FRAME_WIDTH,  cfg_.width);
        capture_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg_.height);
    }
    if (cfg_.fps > 0) {
        capture_.set(cv::CAP_PROP_FPS, cfg_.fps);
    }

    // Log actual settings (may differ from requested)
    double actual_w   = capture_.get(cv::CAP_PROP_FRAME_WIDTH);
    double actual_h   = capture_.get(cv::CAP_PROP_FRAME_HEIGHT);
    double actual_fps = capture_.get(cv::CAP_PROP_FPS);
    logger_.info("Camera opened – resolution: ", static_cast<int>(actual_w),
                 "x", static_cast<int>(actual_h),
                 " @ ", static_cast<int>(actual_fps), " fps");

    running_ = true;
    capture_thread_ = std::thread(&CameraInterface::capture_loop, this);
    return true;
}

void CameraInterface::stop() {
    if (!running_) return;
    running_ = false;
    if (capture_thread_.joinable()) capture_thread_.join();
    capture_.release();
    if (cfg_.show_preview) cv::destroyAllWindows();
    logger_.info("Camera stopped");
}

cv::Mat CameraInterface::get_latest_frame() const {
    std::lock_guard<std::mutex> lk(frame_mtx_);
    new_frame_available_ = false;
    return latest_frame_.clone();
}

void CameraInterface::capture_loop() {
    cv::Mat frame;
    auto    last_fps_time = std::chrono::steady_clock::now();
    int     fps_frame_count = 0;

    while (running_) {
        if (!capture_.read(frame) || frame.empty()) {
            logger_.warn("Empty frame from camera – retrying");
            std::this_thread::sleep_for(10ms);
            continue;
        }

        if (cfg_.flip_horizontal) {
            cv::flip(frame, frame, 1);
        }

        // Update shared latest frame
        {
            std::lock_guard<std::mutex> lk(frame_mtx_);
            latest_frame_ = frame.clone();
        }
        new_frame_available_ = true;
        ++frame_count_;

        // Invoke optional callback (no lock held)
        if (frame_callback_) {
            frame_callback_(frame);
        }

        // Optional preview window
        if (cfg_.show_preview) {
            cv::imshow("Camera Preview", frame);
            if (cv::waitKey(1) == 27) {  // ESC to close
                running_ = false;
                break;
            }
        }

        // Rolling FPS estimate (every 30 frames)
        if (++fps_frame_count >= 30) {
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - last_fps_time).count();
            current_fps_    = fps_frame_count / elapsed;
            fps_frame_count = 0;
            last_fps_time   = now;
        }
    }
}
