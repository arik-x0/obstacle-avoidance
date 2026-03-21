#include "rotation_tracker.hpp"

#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ── update ────────────────────────────────────────────────────────────────────

void RotationTracker::update(const EKFSnapshot& ekf) {
    quat_to_R(ekf.qw, ekf.qx, ekf.qy, ekf.qz, R_);
    yaw_deg_   = ekf.yaw_deg;
    pitch_deg_ = ekf.pitch_deg;
    roll_deg_  = ekf.roll_deg;
}

// ── Transforms ───────────────────────────────────────────────────────────────

void RotationTracker::body_to_ned(float fwd, float right, float down,
                                   float& north, float& east, float& ned_down) const {
    // v_ned = R * v_body
    north    = R_[0][0]*fwd + R_[0][1]*right + R_[0][2]*down;
    east     = R_[1][0]*fwd + R_[1][1]*right + R_[1][2]*down;
    ned_down = R_[2][0]*fwd + R_[2][1]*right + R_[2][2]*down;
}

void RotationTracker::ned_to_body(float north, float east, float ned_down,
                                   float& fwd, float& right, float& down) const {
    // v_body = R^T * v_ned  (R is orthogonal, so R^-1 = R^T)
    fwd   = R_[0][0]*north + R_[1][0]*east + R_[2][0]*ned_down;
    right = R_[0][1]*north + R_[1][1]*east + R_[2][1]*ned_down;
    down  = R_[0][2]*north + R_[1][2]*east + R_[2][2]*ned_down;
}

void RotationTracker::yaw_only_body_to_ned(float fwd, float right,
                                            float& north, float& east) const {
    // Pure yaw rotation – ignores pitch/roll for simple horizontal commands.
    float yaw_rad = static_cast<float>(yaw_deg_ * M_PI / 180.0);
    float c = std::cos(yaw_rad);
    float s = std::sin(yaw_rad);
    north =  c * fwd - s * right;
    east  =  s * fwd + c * right;
}

// ── Yaw tracking ─────────────────────────────────────────────────────────────

void RotationTracker::mark_yaw_start() {
    yaw_mark_ = yaw_deg_;
}

float RotationTracker::delta_yaw_deg() const {
    float d = yaw_deg_ - yaw_mark_;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

// ── Quaternion → rotation matrix ─────────────────────────────────────────────

void RotationTracker::quat_to_R(float qw, float qx, float qy, float qz,
                                  float R[3][3]) {
    // Standard ZYX aerospace convention body→NED:
    //
    //   R = [ 1-2(qy²+qz²)    2(qxqy-qwqz)   2(qxqz+qwqy) ]
    //       [ 2(qxqy+qwqz)    1-2(qx²+qz²)   2(qyqz-qwqx) ]
    //       [ 2(qxqz-qwqy)    2(qyqz+qwqx)   1-2(qx²+qy²) ]

    R[0][0] = 1.0f - 2.0f*(qy*qy + qz*qz);
    R[0][1] =        2.0f*(qx*qy - qw*qz);
    R[0][2] =        2.0f*(qx*qz + qw*qy);

    R[1][0] =        2.0f*(qx*qy + qw*qz);
    R[1][1] = 1.0f - 2.0f*(qx*qx + qz*qz);
    R[1][2] =        2.0f*(qy*qz - qw*qx);

    R[2][0] =        2.0f*(qx*qz - qw*qy);
    R[2][1] =        2.0f*(qy*qz + qw*qx);
    R[2][2] = 1.0f - 2.0f*(qx*qx + qy*qy);
}
