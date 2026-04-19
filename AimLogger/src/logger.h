// logger.h
#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>

enum class LogLevel : uint8_t
{
  DEBUG = 0x01U,
  INFO  = 0x02U,
  WARN  = 0x04U,
  ERROR = 0x08U
};

class Logger final
{
public:
    explicit Logger(Stream& output, uint8_t node_id, LogLevel level = LogLevel::INFO)
  : output_(&output), node_id_(node_id), mask_(static_cast<uint8_t>(level)) {}

  void setLevel(LogLevel level) { mask_ = static_cast<uint8_t>(level); }

    void setFilterMask(uint8_t mask) { mask_ = static_cast<uint8_t>(mask & 0x0FU); }
    uint8_t filterMask() const { return static_cast<uint8_t>(mask_ & 0x0FU); }

    void setOutput(Stream& output) { output_ = &output; }
    void setNodeId(uint8_t node_id) { node_id_ = node_id; }

    void log(LogLevel level, const char* fmt, ...);

private:
    static constexpr size_t kLineSize = 160U;

    static const char* levelToString(LogLevel level);
    bool shouldLog(LogLevel level) const;

    Stream*   output_;
    uint8_t   node_id_;
    uint8_t mask_;
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