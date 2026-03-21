#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <cstddef>

// ── CommandQueue<T> ───────────────────────────────────────────────────────────
//
// Bounded FIFO queue between one (or more) producers and one consumer.
//
// Producer:  push(val)      – enqueues val; drops the oldest entry if full
// Consumer:  pop(out)       – blocks until an item is available or timeout
//            try_pop(out)   – non-blocking pop; returns false if empty
//
// Semantics: capped at max_size entries. When the cap is reached, the oldest
// entry is silently dropped so the producer never blocks. This preserves
// real-time safety: the drone will always act on the latest intent, not stale
// commands that piled up while it was busy.

template<typename T>
class CommandQueue {
public:
    explicit CommandQueue(size_t max_size = 16) : max_size_(max_size) {}

    // Producer side ──────────────────────────────────────────────────────────

    void push(T val) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (q_.size() >= max_size_) q_.pop();   // drop oldest
            q_.push(std::move(val));
        }
        cv_.notify_one();
    }

    // Consumer side ──────────────────────────────────────────────────────────

    // Blocks until an item arrives or timeout elapses.
    // Returns true when an item was dequeued; false on timeout.
    bool pop(T& out,
             std::chrono::milliseconds timeout = std::chrono::milliseconds{500}) {
        std::unique_lock<std::mutex> lk(mtx_);
        bool got = cv_.wait_for(lk, timeout, [this] { return !q_.empty(); });
        if (got) {
            out = std::move(q_.front());
            q_.pop();
        }
        return got;
    }

    // Non-blocking pop. Returns true if an item was dequeued.
    bool try_pop(T& out) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop();
        return true;
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.empty();
    }

private:
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    std::queue<T>           q_;
    size_t                  max_size_;
};
