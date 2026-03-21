#pragma once

#include <string>
#include <thread>
#include <mutex>
#include <atomic>
#include <optional>
#include <functional>

#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>

#include "../utils/logger.hpp"

struct CameraConfig {
    // Source: integer = device index (0=first cam), string = URL / GStreamer pipeline
    int         device_index = 0;
    std::string source_url;          // overrides device_index if non-empty
    int         width  = 640;
    int         height = 480;
    int         fps    = 30;
    bool        flip_horizontal = false;
    bool        show_preview    = false;
};

class CameraInterface {
public:
    explicit CameraInterface(const CameraConfig& cfg, Logger& logger);
    ~CameraInterface();

    // Non-copyable
    CameraInterface(const CameraInterface&) = delete;
    CameraInterface& operator=(const CameraInterface&) = delete;

    // Open the camera and start background capture thread.
    bool start();

    // Stop capture and close device.
    void stop();

    bool is_running() const { return running_; }

    // Returns the latest captured frame (deep copy, thread-safe).
    // Returns empty Mat if no frame has been captured yet.
    cv::Mat get_latest_frame() const;

    // Returns true if a new frame has arrived since the last call to
    // get_latest_frame().
    bool has_new_frame() const { return new_frame_available_; }

    // Optional frame callback – invoked from the capture thread.
    // Useful for low-latency consumers; be careful not to block inside.
    void set_frame_callback(std::function<void(const cv::Mat&)> cb) {
        frame_callback_ = std::move(cb);
    }

    // Statistics
    double current_fps() const { return current_fps_; }
    uint64_t frame_count() const { return frame_count_; }

private:
    void capture_loop();

    CameraConfig cfg_;
    Logger&      logger_;

    cv::VideoCapture capture_;

    mutable std::mutex   frame_mtx_;
    cv::Mat              latest_frame_;
    mutable std::atomic<bool> new_frame_available_{false};

    std::thread          capture_thread_;
    std::atomic<bool>    running_{false};
    std::atomic<double>  current_fps_{0.0};
    std::atomic<uint64_t> frame_count_{0};

    std::function<void(const cv::Mat&)> frame_callback_;
};
