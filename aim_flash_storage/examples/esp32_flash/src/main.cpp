#include <Arduino.h>
#include <math.h>
#include <aim_file_system.h>
#include <aim_flight_recorder.h>
#include <aim_console.h>
#include <logger.h>

static constexpr uint8_t  kNodeId          = 1U;
static constexpr uint8_t  kNumCols         = 4U;
static constexpr uint32_t kRowIntervalMs   = 100U;
static constexpr uint32_t kMaxLogSize      = 10UL * 1024UL * 1024UL;

static const char* const kHeaders[kNumCols] = {"Time", "Counter", "Sine", "Ramp"};
static const char*       kBoardName         = "esp32-flash";

static ESP32PartitionDriver s_flashDriver("storage");
static AimFileSystem        s_fs(&s_flashDriver);
static AimFlightRecorder    s_recorder(s_fs, kNumCols, 100U, kMaxLogSize, kHeaders);
static Logger               s_logger(Serial, kNodeId);

static uint32_t s_rowCounter = 0U;

// --- Application hook ---

static void handleStatus(Stream& out) {
  out.printf("uptime=%lu ms  rows=%lu\r\n",
             static_cast<unsigned long>(millis()),
             static_cast<unsigned long>(s_rowCounter));
}

static const AimConsoleHook kHooks[] = {
  {'s', "status", handleStatus},
};

// --- Telemetry ---

static void serviceLog(void) {
  static uint32_t s_lastMs = 0U;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - s_lastMs) < kRowIntervalMs) { return; }
  s_lastMs = nowMs;

  const float angle = static_cast<float>(nowMs % 10000U) * 3.14159f / 5000.0f;
  uint32_t row[kNumCols];
  row[0] = nowMs;
  row[1] = s_rowCounter;
  row[2] = static_cast<uint32_t>((sinf(angle) + 1.0f) * 10000.0f);
  row[3] = s_rowCounter % 1000U;

  if (!s_recorder.writeRow(row)) { LOG_WARN("writeRow failed"); }
  s_rowCounter++;
}

// --- Arduino ---

void setup() {
  Serial.begin(115200);
  g_logger = &s_logger;

  if (!s_fs.begin()) { LOG_ERROR("Filesystem mount failed"); return; }
  if (!s_recorder.begin()) { LOG_ERROR("Recorder init failed"); return; }

  aimConsoleInit(Serial, s_fs, s_recorder, kBoardName, kHooks, 1U);
  LOG_INFO("ready — send 'd' to enter console");
}

void loop() {
  if (!aimConsoleIsActive()) { serviceLog(); }
  aimConsoleService();
}
