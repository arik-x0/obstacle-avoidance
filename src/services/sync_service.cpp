#include "sync_service.hpp"

#include <chrono>

using namespace std::chrono_literals;

SyncService::SyncService(DataChannel<StampedDetection>& ch_detection,
                          DataChannel<DepthEstimate>&    ch_depth,
                          EKFRingBuffer&                 ekf_ring,
                          DataChannel<SyncedObservation>& ch_synced,
                          Logger&                         logger)
    : ch_detection_(ch_detection)
    , ch_depth_(ch_depth)
    , ekf_ring_(ekf_ring)
    , ch_synced_(ch_synced)
    , logger_(logger)
{}

void SyncService::run() {
    logger_.info("[SyncService] Started");

    uint64_t last_det_seq = 0;

    while (running_) {
        // Trigger on every new detection (depth is always produced alongside it,
        // so reading the latest depth value is correct).
        StampedDetection det;
        if (!ch_detection_.wait_next(det, last_det_seq, 100ms)) {
            continue;
        }
        last_det_seq = ch_detection_.seq();

        // Read latest depth estimate (produced by DepthService from the same detection).
        DepthEstimate depth = ch_depth_.latest();

        // Interpolate EKF state to the frame's capture timestamp.
        EKFSnapshot ekf{};
        if (!ekf_ring_.interpolate(det.timestamp_us, ekf)) {
            // EKF ring not yet populated (startup) – skip this observation.
            continue;
        }

        SyncedObservation obs;
        obs.detection    = det;
        obs.depth        = depth;
        obs.ekf          = ekf;
        obs.timestamp_us = det.timestamp_us;

        ch_synced_.publish(std::move(obs));
    }

    logger_.info("[SyncService] Stopped");
}
