// logger.cpp
#include "logger.h"

Logger* g_logger = nullptr;

const char* Logger::levelToString(LogLevel level)
{
    switch (level)
    {
        case LogLevel::DEBUG: return "DEBUG";
        case LogLevel::INFO:  return "INFO ";
        case LogLevel::WARN:  return "WARN ";
        case LogLevel::ERROR: return "ERROR";
        default:              return "?????";
    }
}

void Logger::logV(LogLevel level, const char* fmt, va_list args)
{
    if ((output_ == nullptr) || (level < level_) || (fmt == nullptr))
    {
        return;
    }

    char msg[kMsgSize];
    char line[kLineSize];

    (void)vsnprintf(msg, sizeof(msg), fmt, args);

    const unsigned long ts = millis();
    (void)snprintf(
        line,
        sizeof(line),
        "[NODE:%u][%08lu][%s] %s",
        static_cast<unsigned>(node_id_),
        ts,
        levelToString(level),
        msg
    );

    output_->println(line);
}

void Logger::log(LogLevel level, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logV(level, fmt, args);
    va_end(args);
}

void Logger::debug(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logV(LogLevel::DEBUG, fmt, args);
    va_end(args);
}

void Logger::info(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logV(LogLevel::INFO, fmt, args);
    va_end(args);
}

void Logger::warn(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logV(LogLevel::WARN, fmt, args);
    va_end(args);
}

void Logger::error(const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    logV(LogLevel::ERROR, fmt, args);
    va_end(args);
}