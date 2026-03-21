#pragma once

#include "../core/messages.hpp"

// ── RotationTracker ───────────────────────────────────────────────────────────
//
// Converts the EKF3 attitude quaternion into a 3×3 rotation matrix and
// exposes body ↔ NED frame transforms.
//
// The rotation matrix R_body_to_ned satisfies:
//
//   v_ned = R * v_body
//
// where v_body is a vector in drone body axes:
//   X = forward (nose)
//   Y = right (starboard)
//   Z = down
//
// and v_ned is in NED (North-East-Down) axes.
//
// Because R comes directly from the EKF3 quaternion, it is re-derived on every
// update() call – no integration drift.  For manoeuvre-completion tracking,
// mark_yaw_start() / delta_yaw_deg() measure accumulated yaw change between
// EKF updates.

class RotationTracker {
public:
    // Called each control tick with the latest EKF snapshot.
    void update(const EKFSnapshot& ekf);

    // Transform body-frame velocity [fwd, right, down] → NED [north, east, down].
    void body_to_ned(float fwd,   float right, float down,
                     float& north, float& east, float& ned_down) const;

    // Transform NED [north, east, down] → body [fwd, right, down].
    // Uses R^T (orthogonal matrix: R^-1 = R^T).
    void ned_to_body(float north, float east, float ned_down,
                     float& fwd, float& right, float& down) const;

    // Convenience: yaw-only rotation for 2-D lateral dodging in the horizontal plane.
    void yaw_only_body_to_ned(float fwd, float right,
                               float& north, float& east) const;

    float yaw_deg()   const { return yaw_deg_; }
    float pitch_deg() const { return pitch_deg_; }
    float roll_deg()  const { return roll_deg_; }

    // Yaw tracking for manoeuvre completion.
    // mark_yaw_start() latches the current yaw.
    // delta_yaw_deg()  returns the signed angular difference since the mark
    //                  (wraps to [-180, +180]).
    void  mark_yaw_start();
    float delta_yaw_deg() const;

private:
    // Build R from unit quaternion (q must be normalised).
    static void quat_to_R(float qw, float qx, float qy, float qz,
                          float R[3][3]);

    // Row-major 3×3 rotation matrix (body → NED).
    float R_[3][3]{};

    float yaw_deg_{0.0f};
    float pitch_deg_{0.0f};
    float roll_deg_{0.0f};

    float yaw_mark_{0.0f};
};
