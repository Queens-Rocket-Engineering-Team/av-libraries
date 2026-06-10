// logger.cpp
#include "logger.h"

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
    if ((output_ == nullptr) || !shouldLog(level) || (fmt == nullptr))
    {
        return;
    }

    char line[kLineSize];
    const unsigned long ts = millis();

    int offset = snprintf(
        line,
        sizeof(line),
        "N%u:%08lu:%s ",
        static_cast<unsigned>(node_id_),
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

    output_->println(line);
}