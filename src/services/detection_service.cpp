#include "detection_service.hpp"

#include <opencv2/imgproc.hpp>
#include <chrono>

using namespace std::chrono_literals;

DetectionService::DetectionService(const DetectorConfig&          cfg,
                                    DataChannel<StampedFrame>&      ch_frame,
                                    DataChannel<StampedDetection>& ch_detection,
                                    Logger&                         logger)
    : detector_(cfg, logger)
    , ch_frame_(ch_frame)
    , ch_detection_(ch_detection)
    , logger_(logger)
{}

void DetectionService::run() {
    logger_.info("[DetectionService] Started");

    uint64_t last_seq = 0;

    while (running_) {
        StampedFrame sf;
        if (!ch_frame_.wait_next(sf, last_seq, 100ms)) {
            continue;
        }
        last_seq = ch_frame_.seq();

        if (sf.frame.empty()) continue;

        StampedDetection sd;
        sd.timestamp_us = sf.timestamp_us;
        sd.result       = detector_.detect(sf.frame);

        compute_sector_histogram(sf.frame, sd);

        ch_detection_.publish(std::move(sd));
    }

    logger_.info("[DetectionService] Stopped");
}

void DetectionService::compute_sector_histogram(const cv::Mat&   frame,
                                                 StampedDetection& out) {
    // Convert to greyscale, blur, then run Canny to get edge map.
    cv::Mat gray, edges;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, gray, {5, 5}, 0);
    cv::Canny(gray, edges, 50, 150);

    // Divide the frame into 8 equal horizontal sectors and measure
    // the fraction of edge pixels in each.
    const int sectors = 8;
    const int w_per   = edges.cols / sectors;

    for (int s = 0; s < sectors; s++) {
        int x0  = s * w_per;
        int x1  = (s < sectors - 1) ? (s + 1) * w_per : edges.cols;
        cv::Rect roi(x0, 0, x1 - x0, edges.rows);

        double mean_val = cv::mean(edges(roi))[0];
        out.sector_edge_density[static_cast<size_t>(s)] =
            static_cast<float>(mean_val / 255.0);
    }
}
