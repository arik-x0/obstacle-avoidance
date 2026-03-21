#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/messages.hpp"
#include "../detection/object_detector.hpp"
#include "../utils/logger.hpp"

// ── DetectionService ──────────────────────────────────────────────────────────
//
// Consumes StampedFrame from ch_frame, runs ObjectDetector on each frame,
// and publishes StampedDetection to ch_detection.
//
// Also computes an 8-sector horizontal edge-density histogram from the Canny
// output.  This data accompanies the DetectionResult into DepthService where
// it forms the spatial "threat histogram".

class DetectionService : public ServiceBase {
public:
    DetectionService(const DetectorConfig&          cfg,
                     DataChannel<StampedFrame>&      ch_frame,
                     DataChannel<StampedDetection>& ch_detection,
                     Logger&                         logger);

    const char* name() const override { return "DetectionService"; }

private:
    void run() override;

    // Compute per-sector edge density from the frame.
    // Fills out.sector_edge_density[0..7] (8 horizontal strips).
    static void compute_sector_histogram(const cv::Mat&   frame,
                                         StampedDetection& out);

    ObjectDetector                  detector_;
    DataChannel<StampedFrame>&      ch_frame_;
    DataChannel<StampedDetection>&  ch_detection_;
    Logger&                         logger_;
};
