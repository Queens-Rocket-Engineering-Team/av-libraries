// AIM Network v0.7.0 minimal example — STM32 publisher + time master.
// Sends a fake altitude ramp at 10 Hz and a TIME sync at 1 Hz, heartbeats via
// service(), and logs heartbeats received from other nodes.

#include <aim_network.h>
#include <aim_job.h>
#include <logger.h>

static constexpr uint32_t kSerialBaud = 115200U;
static constexpr uint32_t kCanBaud = 1000000U;

static aim::Job g_tick100{100U, 0U};
static aim::Job g_tick1000{1000U, 0U};

static constexpr uint8_t kMaxRxFramesPerLoop = 8U;

static Logger g_log(Serial, static_cast<uint8_t>(aim::Source::Ucm), LogLevel::INFO);
static AimCanHardware g_canHw(kCanBaud, CAN1);
static AimNetwork g_aim(&g_canHw, aim::Source::Ucm);

void setup(void) {
  Serial.begin(kSerialBaud);
  g_logger = &g_log;

  if (!g_aim.begin(aim::classBit(aim::Class::Heartbeat))) {
    LOG_ERROR("CAN init failed — halting");
    while (true) {
      delay(1000U);
    }
  }

  LOG_INFO("AIM publisher up, version=%s schema=%u",
           aim::kNetworkVersionString, static_cast<unsigned>(aim::kSchemaVersion));
}

void loop(void) {
  static int32_t altitudeCm = 0;

  const uint32_t nowMs = millis();

  // Bounded RX drain — this node only accepts heartbeats.
  for (uint8_t i = 0U; i < kMaxRxFramesPerLoop; i++) {
    aim::Msg m = {};
    if (!g_aim.receive(m)) {
      break;
    }
    LOG_INFO("Heartbeat: source=%u state=%u schema=%u t=%lu",
             static_cast<unsigned>(m.source),
             static_cast<unsigned>(m.b[0]),
             static_cast<unsigned>(m.b[3]),
             static_cast<unsigned long>(m.timestampMs));
  }

  if (g_tick100.due(nowMs)) {
    altitudeCm += 25;  // fake ramp, meters x100 per catalog

    aim::Msg m = {};
    m.cls = aim::Class::Sensor;
    m.subject = aim::subject::Altitude;
    m.setSensorValue(altitudeCm);
    if (!g_aim.send(m)) {
      LOG_ERROR("Sensor TX failed");
    }
  }

  if (g_tick1000.due(nowMs)) {
    aim::Msg m = {};
    m.cls = aim::Class::Time;
    m.subject = aim::subject::TimeSync;
    // Broadcast clock synchronization baseline
    if (!g_aim.send(m)) {
      LOG_ERROR("Time TX failed");
    }
  }

  g_aim.service(nowMs, aim::NodeState::Nominal, 0U);
  delay(1U);
}
