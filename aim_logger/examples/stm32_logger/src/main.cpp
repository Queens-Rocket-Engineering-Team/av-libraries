#include <Arduino.h>
#include <logger.h>

static constexpr uint8_t kNodeId = 1U;
static Logger s_logger(Serial, kNodeId);  // default mask: INFO only

void setup() {
  Serial.begin(115200);
  g_logger = &s_logger;

  LOG_DEBUG("hidden at startup — INFO mask active");
  LOG_INFO("AimLogger example started  node=%u", kNodeId);
  LOG_WARN("example warning");
  LOG_ERROR("example error");

  s_logger.setFilterMask(
      static_cast<uint8_t>(LogLevel::DEBUG) |
      static_cast<uint8_t>(LogLevel::INFO)  |
      static_cast<uint8_t>(LogLevel::WARN)  |
      static_cast<uint8_t>(LogLevel::ERROR));
  LOG_DEBUG("all levels now enabled");
}

void loop() {
  static uint32_t s_lastMs = 0U;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - s_lastMs) < 2000U) { return; }
  s_lastMs = nowMs;
  LOG_INFO("uptime=%lu ms", static_cast<unsigned long>(nowMs));
}
