#pragma once

#include <deque>

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/messages.hpp"
#include "../utils/logger.hpp"

struct DepthConfig {
    int   window_size        = 10;    // number of frames in the sliding window
    float ttc_max_s          = 30.0f; // cap on TTC output (no-threat sentinel)
    float min_confidence     = 0.3f;  // R² below this → treat as unreliable
    float max_approach_slope = 0.5f;  // coverage/s that maps to approach_speed_norm=1
};

// ── DepthService ──────────────────────────────────────────────────────────────
//
// Estimates proximity from the monocular looming signal.
//
// Algorithm
// ---------
// Maintains a sliding window of (coverage_ratio, timestamp_us) pairs.
// Fits a linear regression over the window:
//
//   coverage(t) ≈ slope * t + intercept
//
// where slope = dA/dt is the rate at which the obstacle grows in the frame.
// Time-to-collision is derived from the looming equation:
//
//   TTC = A(t) / (dA/dt)
//
// Confidence is the R² of the linear fit (0 = noisy / no trend, 1 = clean).
//
// The sector_edge_density from StampedDetection is forwarded directly as the
// spatial threat histogram so AvoidancePlannerService can reason about which
// horizontal sector is most threatening.

class DepthService : public ServiceBase {
public:
    DepthService(const DepthConfig&             cfg,
                 DataChannel<StampedDetection>& ch_detection,
                 DataChannel<DepthEstimate>&    ch_depth,
                 Logger&                        logger);

    const char* name() const override { return "DepthService"; }

private:
    void run() override;

    DepthEstimate compute_estimate(uint64_t timestamp_us,
                                   const std::array<float, 8>& sector_hist);

    struct Sample {
        float    coverage;
        uint64_t ts_us;
    };

    DepthConfig                    cfg_;
    DataChannel<StampedDetection>& ch_detection_;
    DataChannel<DepthEstimate>&    ch_depth_;
    Logger&                        logger_;

    std::deque<Sample> window_;
};
