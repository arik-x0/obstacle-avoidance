#pragma once

#include <mutex>
#include <condition_variable>
#include <cstdint>
#include <chrono>

// ── DataChannel<T> ────────────────────────────────────────────────────────────
//
// Latest-value slot between one publisher and N consumers.
//
// Publisher:  publish(val)          – writes latest, never blocks, notifies all
// Consumer:   latest()              – reads latest immediately (no wait)
//             wait_next(out, seq)   – blocks until a value newer than seq arrives
//             seq()                 – returns the current sequence number
//
// Semantics: if the publisher is faster than a consumer, intermediate values
// are silently overwritten. This is intentional – real-time sensor data should
// always deliver the freshest reading, never back-pressure.

template<typename T>
class DataChannel {
public:
    // Publisher side ─────────────────────────────────────────────────────────

    void publish(T val) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            data_ = std::move(val);
            seq_++;
        }
        cv_.notify_all();
    }

    // Consumer side ──────────────────────────────────────────────────────────

    // Returns the latest value without blocking.
    T latest() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return data_;
    }

    // Blocks until seq() > last_seq or timeout elapses.
    // Returns true when new data arrived; false on timeout.
    // On true, writes the new value to out.
    bool wait_next(T& out, uint64_t last_seq,
                   std::chrono::milliseconds timeout = std::chrono::milliseconds{500}) {
        std::unique_lock<std::mutex> lk(mtx_);
        bool got = cv_.wait_for(lk, timeout, [this, last_seq] {
            return seq_ > last_seq;
        });
        if (got) out = data_;
        return got;
    }

    // Current sequence number (increments on each publish).
    uint64_t seq() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return seq_;
    }

private:
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    T                       data_{};
    uint64_t                seq_{0};
};
