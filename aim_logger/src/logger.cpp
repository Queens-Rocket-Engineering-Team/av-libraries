// logger.cpp
#include "logger.h"

#include <mutex>

// Serializes the shared output so concurrent log lines can't interleave on one
// stream. A no-op on single-threaded bare-metal nodes whose libstdc++ has no threads. 
#if defined(_GLIBCXX_HAS_GTHREADS)
  static std::mutex s_logMutex;
  #define LOGGER_LOCK_OUTPUT() std::lock_guard<std::mutex> s_logGuard(s_logMutex)
#else
  #define LOGGER_LOCK_OUTPUT() ((void)0)
#endif

Logger* g_logger = nullptr;

const char* Logger::levelToString(LogLevel level)
{
    switch (level)
    {
        case LogLevel::DEBUG: return "D";
        case LogLevel::INFO:  return "I";
        case LogLevel::WARN:  return "W";
        case LogLevel::ERROR: return "E";
        default:              return "?";
    }
}

bool Logger::shouldLog(LogLevel level) const
{
    return (filterMask() & static_cast<uint8_t>(level)) != 0U;
}

void Logger::log(LogLevel level, const char* fmt, ...)
{
    if ((_output == nullptr) || !shouldLog(level) || (fmt == nullptr))
    {
        return;
    }

    char line[kLineSize];
    const unsigned long ts = millis();

    int offset = snprintf(
        line,
        sizeof(line),
        "N%u:%08lu:%s ",
        static_cast<unsigned>(_nodeId),
        ts,
        levelToString(level)
    );

    if ((offset > 0) && (static_cast<size_t>(offset) < sizeof(line)))
    {
        va_list args;
        va_start(args, fmt);
        (void)vsnprintf(line + offset, sizeof(line) - static_cast<size_t>(offset), fmt, args);
        va_end(args);
    }

    // Only the shared output needs serializing — the line buffer is local.
    {
        LOGGER_LOCK_OUTPUT();
        _output->println(line);
    }
}
