#pragma once

#include <atomic>
#include <thread>

// ── IService ──────────────────────────────────────────────────────────────────
//
// Pure interface every service implements.

class IService {
public:
    virtual ~IService() = default;

    virtual void        start()       = 0;
    virtual void        stop()        = 0;
    virtual bool        is_running()  const = 0;
    virtual const char* name()        const = 0;
};

// ── ServiceBase ───────────────────────────────────────────────────────────────
//
// CRTP-free base class that handles thread lifetime.
// Subclasses override run(), which executes on a dedicated thread and should
// loop until running_.load() returns false.
//
// start() – spawns the thread (idempotent)
// stop()  – clears running_, joins the thread

class ServiceBase : public IService {
public:
    void start() override {
        if (running_.exchange(true)) return;   // already running
        thread_ = std::thread([this] { run(); });
    }

    void stop() override {
        running_.store(false);
        if (thread_.joinable()) thread_.join();
    }

    bool is_running() const override { return running_.load(); }

protected:
    // Subclasses implement the service loop here.
    // Loop until !running_.
    virtual void run() = 0;

    std::atomic<bool> running_{false};

private:
    std::thread thread_;
};
