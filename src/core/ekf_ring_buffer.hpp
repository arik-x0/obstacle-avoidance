#pragma once

#include <array>
#include <mutex>
#include <cstddef>
#include <cmath>
#include "messages.hpp"

// ── EKFRingBuffer ─────────────────────────────────────────────────────────────
//
// Thread-safe circular buffer of EKFSnapshot entries used by SyncService.
//
// TelemetryService pushes new snapshots at ~20-50 Hz.
// SyncService calls interpolate(timestamp_us) to retrieve the EKF state that
// best corresponds to a camera frame's capture timestamp:
//   - finds the two snapshots bracketing the requested timestamp
//   - linearly interpolates scalars (vel, alt, Euler angles)
//   - spherically interpolates the attitude quaternion (slerp)

class EKFRingBuffer {
public:
    static constexpr size_t CAPACITY = 128;  // ~6 s at 20 Hz

    void push(const EKFSnapshot& snap) {
        std::lock_guard<std::mutex> lk(mtx_);
        buf_[head_] = snap;
        head_       = (head_ + 1) % CAPACITY;
        if (size_ < CAPACITY) size_++;
    }

    // Retrieve an EKFSnapshot interpolated to timestamp_us.
    // Returns false only if the buffer is empty.
    bool interpolate(uint64_t timestamp_us, EKFSnapshot& out) const {
        std::lock_guard<std::mutex> lk(mtx_);

        if (size_ == 0) return false;
        if (size_ == 1) { out = at(0); return true; }

        // Find the closest bracket: lo <= timestamp_us < hi
        size_t best_lo_idx = 0, best_hi_idx = 0;
        bool   found_lo    = false, found_hi = false;

        for (size_t i = 0; i < size_; i++) {
            const EKFSnapshot& s = at(i);
            if (s.timestamp_us <= timestamp_us) {
                if (!found_lo || s.timestamp_us > at(best_lo_idx).timestamp_us) {
                    best_lo_idx = i;
                    found_lo    = true;
                }
            } else {
                if (!found_hi || s.timestamp_us < at(best_hi_idx).timestamp_us) {
                    best_hi_idx = i;
                    found_hi    = true;
                }
            }
        }

        if (!found_lo && !found_hi) { out = at(0); return true; }
        if (!found_lo)  { out = at(best_hi_idx); return true; }
        if (!found_hi)  { out = at(best_lo_idx); return true; }

        const EKFSnapshot& lo = at(best_lo_idx);
        const EKFSnapshot& hi = at(best_hi_idx);

        uint64_t span = hi.timestamp_us - lo.timestamp_us;
        if (span == 0) { out = lo; return true; }

        float t = static_cast<float>(timestamp_us - lo.timestamp_us) /
                  static_cast<float>(span);
        t = (t < 0.0f) ? 0.0f : (t > 1.0f ? 1.0f : t);

        out              = lo;
        out.timestamp_us = timestamp_us;

        // Lerp scalars
        out.vel_north  = lerp(lo.vel_north,  hi.vel_north,  t);
        out.vel_east   = lerp(lo.vel_east,   hi.vel_east,   t);
        out.vel_down   = lerp(lo.vel_down,   hi.vel_down,   t);
        out.alt_rel_m  = lerp(lo.alt_rel_m,  hi.alt_rel_m,  t);
        out.roll_deg   = lerp(lo.roll_deg,   hi.roll_deg,   t);
        out.pitch_deg  = lerp(lo.pitch_deg,  hi.pitch_deg,  t);

        // Yaw wrap-around aware lerp
        float dyaw = hi.yaw_deg - lo.yaw_deg;
        if (dyaw >  180.0f) dyaw -= 360.0f;
        if (dyaw < -180.0f) dyaw += 360.0f;
        out.yaw_deg = lo.yaw_deg + t * dyaw;

        // Quaternion slerp
        slerp(lo.qw, lo.qx, lo.qy, lo.qz,
              hi.qw, hi.qx, hi.qy, hi.qz,
              t,
              out.qw, out.qx, out.qy, out.qz);

        // Status flags: use hi (most recent)
        out.armed  = hi.armed;
        out.in_air = hi.in_air;

        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return size_;
    }

private:
    // Access entries in chronological order (oldest = index 0)
    const EKFSnapshot& at(size_t i) const {
        size_t start = (head_ + CAPACITY - size_) % CAPACITY;
        return buf_[(start + i) % CAPACITY];
    }

    static float lerp(float a, float b, float t) { return a + t * (b - a); }

    static void slerp(float aw, float ax, float ay, float az,
                      float bw, float bx, float by, float bz,
                      float t,
                      float& rw, float& rx, float& ry, float& rz) {
        float dot = aw*bw + ax*bx + ay*by + az*bz;

        // Ensure shortest arc
        if (dot < 0.0f) { bw=-bw; bx=-bx; by=-by; bz=-bz; dot=-dot; }

        // Linear interpolation when quaternions are nearly parallel (avoids div/0)
        if (dot > 0.9995f) {
            rw = aw + t*(bw-aw);
            rx = ax + t*(bx-ax);
            ry = ay + t*(by-ay);
            rz = az + t*(bz-az);
            float len = std::sqrt(rw*rw + rx*rx + ry*ry + rz*rz);
            if (len > 1e-6f) { rw/=len; rx/=len; ry/=len; rz/=len; }
            return;
        }

        float theta0 = std::acos(dot);
        float theta  = theta0 * t;
        float sin0   = std::sin(theta0);
        float s0     = std::cos(theta) - dot * std::sin(theta) / sin0;
        float s1     = std::sin(theta) / sin0;

        rw = s0*aw + s1*bw;
        rx = s0*ax + s1*bx;
        ry = s0*ay + s1*by;
        rz = s0*az + s1*bz;
    }

    mutable std::mutex                mtx_;
    std::array<EKFSnapshot, CAPACITY> buf_{};
    size_t                            head_{0};
    size_t                            size_{0};
};
