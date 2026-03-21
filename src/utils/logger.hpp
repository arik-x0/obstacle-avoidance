#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <mutex>
#include <chrono>
#include <iomanip>
#include <functional>

enum class LogLevel { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3 };

inline const char* level_str(LogLevel l) {
    switch (l) {
        case LogLevel::DEBUG: return "\033[36mDEBUG\033[0m";
        case LogLevel::INFO:  return "\033[32mINFO \033[0m";
        case LogLevel::WARN:  return "\033[33mWARN \033[0m";
        case LogLevel::ERROR: return "\033[31mERROR\033[0m";
    }
    return "?????";
}

class Logger {
public:
    explicit Logger(const std::string& tag, LogLevel min_level = LogLevel::DEBUG)
        : tag_(tag), min_level_(min_level) {}

    void set_level(LogLevel l)  { min_level_ = l; }
    void set_file(const std::string& path) {
        std::lock_guard<std::mutex> lock(mutex_);
        file_.open(path, std::ios::app);
    }

    template<typename... Args>
    void debug(Args&&... args) { log(LogLevel::DEBUG, std::forward<Args>(args)...); }

    template<typename... Args>
    void info(Args&&... args)  { log(LogLevel::INFO,  std::forward<Args>(args)...); }

    template<typename... Args>
    void warn(Args&&... args)  { log(LogLevel::WARN,  std::forward<Args>(args)...); }

    template<typename... Args>
    void error(Args&&... args) { log(LogLevel::ERROR, std::forward<Args>(args)...); }

private:
    template<typename... Args>
    void log(LogLevel level, Args&&... args) {
        if (level < min_level_) return;

        auto now = std::chrono::system_clock::now();
        auto t   = std::chrono::system_clock::to_time_t(now);
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       now.time_since_epoch()) % 1000;

        std::ostringstream oss;
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        oss << std::put_time(&tm_buf, "%H:%M:%S")
            << '.' << std::setw(3) << std::setfill('0') << ms.count()
            << " [" << level_str(level) << "] "
            << '[' << tag_ << "] ";
        (oss << ... << std::forward<Args>(args));

        std::lock_guard<std::mutex> lock(mutex_);
        std::cout << oss.str() << '\n';
        if (file_.is_open()) file_ << oss.str() << '\n';
    }

    std::string   tag_;
    LogLevel      min_level_;
    std::mutex    mutex_;
    std::ofstream file_;
};
