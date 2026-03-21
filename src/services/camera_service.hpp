#pragma once

#include "../core/service_base.hpp"
#include "../core/channel.hpp"
#include "../core/messages.hpp"
#include "../camera/camera_interface.hpp"
#include "../utils/logger.hpp"

// ── CameraService ─────────────────────────────────────────────────────────────
//
// Wraps CameraInterface in a service thread.
// Publishes StampedFrame to ch_frame whenever a new frame is available.
// The timestamp is set to now_us() at the moment the frame is read from the
// capture buffer – close to the actual sensor capture time.

class CameraService : public ServiceBase {
public:
    CameraService(const CameraConfig&        cfg,
                  DataChannel<StampedFrame>& ch_frame,
                  Logger&                    logger);

    void        stop()  override;
    const char* name()  const override { return "CameraService"; }

private:
    void run() override;

    CameraInterface            camera_;
    DataChannel<StampedFrame>& ch_frame_;
    Logger&                    logger_;
};
