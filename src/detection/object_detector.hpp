#pragma once

#include <string>
#include <vector>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/video/background_segm.hpp>

#include "../utils/logger.hpp"
#include "../core/messages.hpp"

struct DetectorConfig {
    // ── Edge / contour detector ─────────────────────────────────────────────
    int   canny_low      = 50;
    int   canny_high     = 150;
    int   blur_kernel    = 5;        // Gaussian blur kernel (must be odd)

    // Contour filtering
    double min_area_ratio = 0.005;   // min contour area / frame area
    double max_area_ratio = 0.90;    // ignore near-full-frame contours (e.g. border)

    // ── Background subtractor (motion-based) ────────────────────────────────
    bool  use_background_sub = false; // true = MOG2, false = edge-based
    int   bg_history         = 200;
    double bg_var_threshold  = 16.0;

    // ── Sector thresholds ───────────────────────────────────────────────────
    float left_sector_end   = 0.35f; // normalised X
    float right_sector_start = 0.65f;

    // ── Visualisation ───────────────────────────────────────────────────────
    bool  draw_debug        = false;  // annotate frame with detections
};

class ObjectDetector {
public:
    explicit ObjectDetector(const DetectorConfig& cfg, Logger& logger);

    // Process one frame and return a DetectionResult.
    // Optionally writes an annotated frame to debug_frame if draw_debug=true.
    DetectionResult detect(const cv::Mat& frame,
                           cv::Mat* debug_frame = nullptr);

    // Dynamically update thresholds at runtime
    void set_canny_thresholds(int low, int high);
    void reset_background_model();

private:
    DetectionResult detect_edges(const cv::Mat& frame, cv::Mat* dbg);
    DetectionResult detect_motion(const cv::Mat& frame, cv::Mat* dbg);

    static DetectionResult::Sector classify_sector(
        float centre_x_norm,
        float left_end,
        float right_start);

    DetectorConfig cfg_;
    Logger&        logger_;

    cv::Ptr<cv::BackgroundSubtractorMOG2> bg_sub_;
};
