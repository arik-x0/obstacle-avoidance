#include "object_detector.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <algorithm>

ObjectDetector::ObjectDetector(const DetectorConfig& cfg, Logger& logger)
    : cfg_(cfg), logger_(logger)
{
    if (cfg_.use_background_sub) {
        bg_sub_ = cv::createBackgroundSubtractorMOG2(
            cfg_.bg_history, cfg_.bg_var_threshold, /*detectShadows=*/false);
        logger_.info("[Detector] Using background subtractor (MOG2)");
    } else {
        logger_.info("[Detector] Using edge / contour detection (Canny)");
    }
}

DetectionResult ObjectDetector::detect(const cv::Mat& frame, cv::Mat* dbg) {
    if (frame.empty()) {
        logger_.warn("[Detector] Received empty frame");
        return {};
    }

    if (cfg_.use_background_sub) {
        return detect_motion(frame, dbg);
    }
    return detect_edges(frame, dbg);
}

// ── Edge-based detection ──────────────────────────────────────────────────────

DetectionResult ObjectDetector::detect_edges(const cv::Mat& frame, cv::Mat* dbg) {
    cv::Mat gray, blurred, edges;

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred,
                     cv::Size(cfg_.blur_kernel, cfg_.blur_kernel), 0);
    cv::Canny(blurred, edges, cfg_.canny_low, cfg_.canny_high);

    // Morphological close to join nearby edges into solid regions
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = static_cast<double>(frame.cols) * frame.rows;
    const double min_area   = cfg_.min_area_ratio * frame_area;
    const double max_area   = cfg_.max_area_ratio * frame_area;

    // Find the largest qualifying contour
    cv::Rect   best_bbox;
    double     best_area = 0.0;

    for (const auto& contour : contours) {
        double area = cv::contourArea(contour);
        if (area < min_area || area > max_area) continue;

        if (area > best_area) {
            best_area = area;
            best_bbox = cv::boundingRect(contour);
        }
    }

    DetectionResult result;

    if (best_area > 0.0) {
        result.obstacle_detected = true;
        result.bounding_box      = best_bbox;
        result.coverage_ratio    = static_cast<float>(best_area / frame_area);
        result.centre_x_norm     = static_cast<float>(best_bbox.x + best_bbox.width / 2.0)
                                   / static_cast<float>(frame.cols);
        result.sector            = classify_sector(result.centre_x_norm,
                                                   cfg_.left_sector_end,
                                                   cfg_.right_sector_start);
    }

    // Debug visualisation
    if (dbg && cfg_.draw_debug) {
        frame.copyTo(*dbg);
        if (result.obstacle_detected) {
            // Draw bounding box
            cv::Scalar color;
            switch (result.threat()) {
                case DetectionResult::Threat::FAR:      color = cv::Scalar(0,255,0);   break;
                case DetectionResult::Threat::MEDIUM:   color = cv::Scalar(0,165,255); break;
                case DetectionResult::Threat::CLOSE:    color = cv::Scalar(0,0,255);   break;
                case DetectionResult::Threat::CRITICAL: color = cv::Scalar(0,0,180);   break;
                default:                                color = cv::Scalar(200,200,200); break;
            }
            cv::rectangle(*dbg, result.bounding_box, color, 2);

            // Threat label
            std::string label = "OBSTACLE";
            switch (result.threat()) {
                case DetectionResult::Threat::MEDIUM:   label += " [MED]";    break;
                case DetectionResult::Threat::CLOSE:    label += " [CLOSE]";  break;
                case DetectionResult::Threat::CRITICAL: label += " [CRIT!]";  break;
                default: break;
            }
            cv::putText(*dbg, label,
                        cv::Point(result.bounding_box.x, result.bounding_box.y - 5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
        }

        // Sector dividers
        int lx = static_cast<int>(cfg_.left_sector_end   * frame.cols);
        int rx = static_cast<int>(cfg_.right_sector_start * frame.cols);
        cv::line(*dbg, cv::Point(lx, 0), cv::Point(lx, frame.rows),
                 cv::Scalar(255,255,0), 1, cv::LINE_AA);
        cv::line(*dbg, cv::Point(rx, 0), cv::Point(rx, frame.rows),
                 cv::Scalar(255,255,0), 1, cv::LINE_AA);

        // Coverage ratio HUD
        std::string hud = "Cover: " + std::to_string(int(result.coverage_ratio * 100)) + "%";
        cv::putText(*dbg, hud, cv::Point(10, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(200,200,200), 1);
    }

    return result;
}

// ── Motion-based detection (MOG2) ────────────────────────────────────────────

DetectionResult ObjectDetector::detect_motion(const cv::Mat& frame, cv::Mat* dbg) {
    cv::Mat fg_mask;
    bg_sub_->apply(frame, fg_mask);

    // Morphological opening to remove noise, closing to fill gaps
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_OPEN,  kernel);
    cv::morphologyEx(fg_mask, fg_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(fg_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = static_cast<double>(frame.cols) * frame.rows;
    const double min_area   = cfg_.min_area_ratio * frame_area;
    const double max_area   = cfg_.max_area_ratio * frame_area;

    cv::Rect   best_bbox;
    double     best_area = 0.0;

    for (const auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < min_area || area > max_area) continue;
        if (area > best_area) {
            best_area = area;
            best_bbox = cv::boundingRect(c);
        }
    }

    DetectionResult result;

    if (best_area > 0.0) {
        result.obstacle_detected = true;
        result.bounding_box      = best_bbox;
        result.coverage_ratio    = static_cast<float>(best_area / frame_area);
        result.centre_x_norm     = static_cast<float>(best_bbox.x + best_bbox.width / 2.0)
                                   / static_cast<float>(frame.cols);
        result.sector            = classify_sector(result.centre_x_norm,
                                                   cfg_.left_sector_end,
                                                   cfg_.right_sector_start);
    }

    if (dbg && cfg_.draw_debug) {
        cv::cvtColor(fg_mask, *dbg, cv::COLOR_GRAY2BGR);
        if (result.obstacle_detected) {
            cv::rectangle(*dbg, result.bounding_box, cv::Scalar(0,0,255), 2);
        }
    }

    return result;
}

// ── Helpers ───────────────────────────────────────────────────────────────────

DetectionResult::Sector ObjectDetector::classify_sector(float cx,
                                                         float left_end,
                                                         float right_start) {
    if (cx < left_end)    return DetectionResult::Sector::LEFT;
    if (cx > right_start) return DetectionResult::Sector::RIGHT;
    return DetectionResult::Sector::CENTER;
}

void ObjectDetector::set_canny_thresholds(int low, int high) {
    cfg_.canny_low  = low;
    cfg_.canny_high = high;
}

void ObjectDetector::reset_background_model() {
    if (bg_sub_) {
        bg_sub_ = cv::createBackgroundSubtractorMOG2(
            cfg_.bg_history, cfg_.bg_var_threshold, false);
        logger_.info("[Detector] Background model reset");
    }
}
