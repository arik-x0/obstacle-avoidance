#include "depth_service.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

DepthService::DepthService(const DepthConfig&             cfg,
                            DataChannel<StampedDetection>& ch_detection,
                            DataChannel<DepthEstimate>&    ch_depth,
                            Logger&                        logger)
    : cfg_(cfg)
    , ch_detection_(ch_detection)
    , ch_depth_(ch_depth)
    , logger_(logger)
{}

void DepthService::run() {
    logger_.info("[DepthService] Started");

    uint64_t last_seq = 0;

    while (running_) {
        StampedDetection sd;
        if (!ch_detection_.wait_next(sd, last_seq, 100ms)) {
            continue;
        }
        last_seq = ch_detection_.seq();

        // Feed the sliding window with the latest coverage sample.
        window_.push_back({sd.result.coverage_ratio, sd.timestamp_us});
        while (static_cast<int>(window_.size()) > cfg_.window_size) {
            window_.pop_front();
        }

        DepthEstimate de = compute_estimate(sd.timestamp_us,
                                            sd.sector_edge_density);
        ch_depth_.publish(de);
    }

    logger_.info("[DepthService] Stopped");
}

DepthEstimate DepthService::compute_estimate(uint64_t                     timestamp_us,
                                              const std::array<float, 8>& sector_hist) {
    DepthEstimate de{};
    de.timestamp_us      = timestamp_us;
    de.sector_histogram  = sector_hist;
    de.ttc_s             = cfg_.ttc_max_s;
    de.approach_speed_norm = 0.0f;
    de.confidence          = 0.0f;

    if (static_cast<int>(window_.size()) < 2) {
        return de;
    }

    // ── Linear regression: coverage = slope * t + intercept ─────────────────
    //
    // t is expressed in seconds relative to the oldest sample to avoid
    // floating-point precision loss with large timestamp values.

    float t0  = static_cast<float>(window_.front().ts_us) * 1e-6f;
    float n   = static_cast<float>(window_.size());

    float sum_t  = 0.0f, sum_c  = 0.0f;
    float sum_tt = 0.0f, sum_tc = 0.0f;

    for (const auto& s : window_) {
        float t = static_cast<float>(s.ts_us) * 1e-6f - t0;
        float c = s.coverage;
        sum_t  += t;
        sum_c  += c;
        sum_tt += t * t;
        sum_tc += t * c;
    }

    float denom = n * sum_tt - sum_t * sum_t;
    if (std::abs(denom) < 1e-9f) {
        return de;  // all samples at the same timestamp – skip
    }

    float slope     = (n * sum_tc - sum_t * sum_c) / denom;   // dA/dt
    float intercept = (sum_c  - slope * sum_t) / n;

    // ── TTC from looming equation ─────────────────────────────────────────────
    float current_cov = window_.back().coverage;

    if (slope > 1e-5f && current_cov > 0.001f) {
        de.ttc_s = std::min(current_cov / slope, cfg_.ttc_max_s);
    }

    // ── Normalised approach speed ─────────────────────────────────────────────
    if (slope > 0.0f) {
        de.approach_speed_norm =
            std::min(1.0f, slope / cfg_.max_approach_slope);
    }

    // ── R² confidence ─────────────────────────────────────────────────────────
    float mean_c = sum_c / n;
    float ss_tot = 0.0f, ss_res = 0.0f;

    for (const auto& s : window_) {
        float t         = static_cast<float>(s.ts_us) * 1e-6f - t0;
        float predicted = slope * t + intercept;
        float residual  = s.coverage - predicted;
        float total     = s.coverage - mean_c;
        ss_res += residual * residual;
        ss_tot += total   * total;
    }

    de.confidence = (ss_tot > 1e-9f)
                        ? std::max(0.0f, 1.0f - ss_res / ss_tot)
                        : 0.0f;

    return de;
}
