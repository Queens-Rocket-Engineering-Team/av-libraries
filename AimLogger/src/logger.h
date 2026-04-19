// logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

enum class LogLevel : uint8_t
{
    DEBUG = 0U,
    INFO  = 1U,
    WARN  = 2U,
    ERROR = 3U
};

class Logger final
{
public:
    explicit Logger(Stream& output, uint8_t node_id, LogLevel level = LogLevel::INFO)
        : output_(&output), node_id_(node_id), level_(level) {}

    void setLevel(LogLevel level) { level_ = level; }
    LogLevel level() const { return level_; }

    void setOutput(Stream& output) { output_ = &output; }
    void setNodeId(uint8_t node_id) { node_id_ = node_id; }

    void log(LogLevel level, const char* fmt, ...);

private:
    static constexpr size_t kLineSize = 160U;

    static const char* levelToString(LogLevel level);

    Stream*   output_;
    uint8_t   node_id_;
    LogLevel  level_;
};

extern Logger* g_logger;

#ifndef FLIGHT_BUILD
  #define LOG_DEBUG(...) do { if (g_logger) g_logger->log(LogLevel::DEBUG, __VA_ARGS__); } while (0)
  #define LOG_INFO(...)  do { if (g_logger) g_logger->log(LogLevel::INFO, __VA_ARGS__); } while (0)
  #define LOG_WARN(...)  do { if (g_logger) g_logger->log(LogLevel::WARN, __VA_ARGS__); } while (0)
  #define LOG_ERROR(...) do { if (g_logger) g_logger->log(LogLevel::ERROR, __VA_ARGS__); } while (0)
#else
  #define LOG_DEBUG(...) do {} while (0)
  #define LOG_INFO(...)  do {} while (0)
  #define LOG_WARN(...)  do {} while (0)
  #define LOG_ERROR(...) do {} while (0)
#endif

#endif // LOGGER_H