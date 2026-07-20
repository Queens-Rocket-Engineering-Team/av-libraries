// AIM Network v0.6.x minimal example — ESP32 listener.
// Receives sensor, time, and heartbeat frames and logs them; heartbeats itself
// via service(). Pairs with the STM32 publisher example.

#include <aim_can_driver.h>
#include <aim_network.h>
#include <logger.h>

static constexpr uint32_t kSerialBaud = 115200U;
static constexpr uint32_t kCanBaud = 500000U;
static constexpr int kCanRxPin = 2;
static constexpr int kCanTxPin = 1;

static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

static Logger g_log(Serial, static_cast<uint8_t>(aim::Source::Lcm), LogLevel::INFO);
static AimCanDriver g_canHw(kCanBaud, kCanRxPin, kCanTxPin);
static AimNetwork g_aim(&g_canHw, aim::Source::Lcm);

void setup(void) {
  Serial.begin(kSerialBaud);
  g_logger = &g_log;

  const uint16_t classMask = aim::classBit(aim::Class::Sensor) |
                             aim::classBit(aim::Class::Time) |
                             aim::classBit(aim::Class::Heartbeat);
  if (!g_aim.begin(classMask)) {
    LOG_ERROR("CAN init failed — halting");
    while (true) {
      delay(1000U);
    }
  }

  LOG_INFO("AIM listener up, version=%s schema=%u",
           aim::kNetworkVersionString, static_cast<unsigned>(aim::kSchemaVersion));
}

void loop(void) {
  const uint32_t nowMs = millis();

  // Bounded RX drain.
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Msg m = {};
    if (!g_aim.receive(m)) {
      break;
    }

    switch (m.cls) {
      case aim::Class::Sensor:
        LOG_INFO("Sensor: subject=0x%02X value=%ld t=%lu",
                 static_cast<unsigned>(m.subject),
                 static_cast<long>(m.sensorValue()),
                 static_cast<unsigned long>(m.timestampMs));
        break;

      case aim::Class::Time:
        // Clock already synced by receive(); logged here for visibility.
        LOG_INFO("Time sync: synced=%lu",
                 static_cast<unsigned long>(g_aim.syncedMillis()));
        break;

      case aim::Class::Heartbeat:
        LOG_INFO("Heartbeat: source=%u state=%u schema=%u",
                 static_cast<unsigned>(m.source),
                 static_cast<unsigned>(m.b[0]),
                 static_cast<unsigned>(m.b[3]));
        break;

      default:
        break;
    }
  }

  g_aim.service(nowMs, aim::NodeState::Nominal, 0U);
  delay(1U);
}
