#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/ekf_ring_buffer.hpp"
#include "../core/messages.hpp"
#include "../utils/logger.hpp"

// ── SyncService ───────────────────────────────────────────────────────────────
//
// Pairs camera-derived data (detection + depth estimate) with the EKF3 state
// that was active at the frame's capture timestamp.
//
// The EKF runs at ~20-50 Hz, the camera at ~30 Hz, and they are
// asynchronous.  SyncService calls EKFRingBuffer::interpolate() with the
// detection's timestamp to obtain the closest EKF snapshot, using slerp on
// the attitude quaternion.
//
// Output: DataChannel<SyncedObservation> consumed by AvoidancePlannerService.
//
// Trigger: every new DepthEstimate (i.e. every new detection).

class SyncService : public ServiceBase {
public:
    SyncService(DataChannel<StampedDetection>& ch_detection,
                DataChannel<DepthEstimate>&    ch_depth,
                EKFRingBuffer&                 ekf_ring,
                DataChannel<SyncedObservation>& ch_synced,
                Logger&                         logger);

    const char* name() const override { return "SyncService"; }

private:
    void run() override;

    DataChannel<StampedDetection>&  ch_detection_;
    DataChannel<DepthEstimate>&     ch_depth_;
    EKFRingBuffer&                  ekf_ring_;
    DataChannel<SyncedObservation>& ch_synced_;
    Logger&                         logger_;
};
