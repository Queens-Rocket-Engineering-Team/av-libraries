#include <Arduino.h>
#include <math.h>
#include <aim_file_system.h>
#include <aim_flight_recorder.h>
#include <aim_console.h>
#include <logger.h>

static constexpr uint8_t  kNodeId        = 1U;
static constexpr uint8_t  kNumCols       = 16U; // Full 16-column maximum schema
static constexpr uint32_t kRowIntervalMs = 10U;  // 100 Hz high-frequency flight logging
static constexpr uint32_t kMaxLogSize    = 1UL * 1024UL * 1024UL; // 1 MB log cap on 128KB/16MB flash

static const char* const kHeaders[kNumCols] = {
  "Time",     "Counter",  "State",    "Flags",
  "AccelX",   "AccelY",   "AccelZ",   "GyroX",
  "GyroY",    "GyroZ",    "BaroPress","BaroTemp",
  "GPS_Lat",  "GPS_Lon",  "GPS_Alt",  "BattVolt"
};

static const char* kBoardName = "stm32-flash-stress";

// SPI2: MOSI=PB15, MISO=PB14, SCLK=PB13, CS=PB12
static SPIClass          s_spi2(PB15, PB14, PB13);
static SpiNorFlashDriver s_flashDriver(PB12, s_spi2);
static AimFileSystem     s_fs(&s_flashDriver);
static AimFlightRecorder s_recorder(s_fs, kNumCols, 100U, kMaxLogSize, kHeaders);
static Logger            s_logger(Serial, kNodeId);

static uint32_t s_rowCounter = 0U;
static uint32_t s_writeFailures = 0U;

// --- Application Hooks ---

static void handleStatus(Stream& out) {
  out.printf("uptime=%lu ms  rows=%lu  fails=%lu  cols=%u  rate=100Hz\r\n",
             static_cast<unsigned long>(millis()),
             static_cast<unsigned long>(s_rowCounter),
             static_cast<unsigned long>(s_writeFailures),
             kNumCols);
}

static const AimConsoleHook kHooks[] = {
  {'s', "status", handleStatus},
};

// --- Stress Telemetry Generator ---

static void serviceLog(void) {
  static uint32_t s_lastMs = 0U;
  const uint32_t nowMs = millis();
  if ((uint32_t)(nowMs - s_lastMs) < kRowIntervalMs) { return; }
  s_lastMs = nowMs;

  const float t = static_cast<float>(nowMs) * 0.001f;

  uint32_t row[kNumCols];

  // Level 2 (2 bytes): Time delta ~10 ms
  row[0] = nowMs;

  // Level 1 (1 byte): Counter delta +1
  row[1] = s_rowCounter;

  // Level 1 (1 byte): State enum (slow transition every 500 rows)
  row[2] = (s_rowCounter / 500U) % 6U;

  // Level 1 (1 byte): Bitmask flags (toggle bit 0 every 100 rows)
  row[3] = (s_rowCounter % 100U == 0) ? 0x01U : 0x00U;

  // Level 2 (2 bytes): AccelX noise (+- 200 mG)
  int32_t ax = static_cast<int32_t>(sinf(t * 12.0f) * 200.0f);
  row[4] = AimFlightRecorder::unsignify(ax);

  // Level 2 (2 bytes): AccelY noise (+- 200 mG)
  int32_t ay = static_cast<int32_t>(cosf(t * 12.0f) * 200.0f);
  row[5] = AimFlightRecorder::unsignify(ay);

  // Level 3 (3 bytes): AccelZ high-G boost spike (+- 5000 mG)
  int32_t az = 1000 + static_cast<int32_t>(sinf(t * 3.0f) * 5000.0f);
  row[6] = AimFlightRecorder::unsignify(az);

  // Level 2 (2 bytes): GyroX (+- 300 dps)
  int32_t gx = static_cast<int32_t>(sinf(t * 25.0f) * 300.0f);
  row[7] = AimFlightRecorder::unsignify(gx);

  // Level 2 (2 bytes): GyroY (+- 300 dps)
  int32_t gy = static_cast<int32_t>(cosf(t * 25.0f) * 300.0f);
  row[8] = AimFlightRecorder::unsignify(gy);

  // Level 3 (3 bytes): GyroZ spin (+- 1500 dps)
  int32_t gz = static_cast<int32_t>(sinf(t * 0.5f) * 1500.0f);
  row[9] = AimFlightRecorder::unsignify(gz);

  // Level 2 (2 bytes): Barometric Pressure (101325 Pa + noise)
  row[10] = 101325U + static_cast<uint32_t>((s_rowCounter % 50U));

  // Level 1 (1 byte): Barometric Temp (2500 = 25.00 C, tiny drift)
  row[11] = 2500U + ((s_rowCounter / 10U) % 3U);

  // Raw 31 (4 bytes): GPS Latitude fixed-point (37.774929 N)
  row[12] = 37774929U + (s_rowCounter % 10U);

  // Raw 32 (5 bytes): GPS Longitude negative fixed-point (-122.419416 W)
  int32_t lon = -122419416 + static_cast<int32_t>(s_rowCounter % 10U);
  row[13] = AimFlightRecorder::unsignify(lon);

  // Level 2 (2 bytes): GPS Altitude (150000 mm + ascent)
  row[14] = 150000U + (s_rowCounter * 5U);

  // Level 1 (1 byte): Battery Voltage (3300 mV - slow drain)
  row[15] = 3300U - ((s_rowCounter / 100U) % 10U);

  if (!s_recorder.writeRow(row)) {
    s_writeFailures++;
    LOG_WARN("writeRow failed (total fails=%lu)", static_cast<unsigned long>(s_writeFailures));
  }
  s_rowCounter++;
}

// --- Arduino Main ---

void setup() {
  Serial.begin(115200);
  g_logger = &s_logger;

  if (!s_fs.begin()) { LOG_ERROR("Filesystem mount failed"); return; }
  if (!s_recorder.begin()) { LOG_ERROR("Recorder init failed"); return; }

  aimConsoleInit(Serial, s_fs, s_recorder, kBoardName, kHooks, 1U);
  LOG_INFO("RDES Stress Test Ready — send 'd' to enter console");
}

void loop() {
  if (!aimConsoleIsActive()) { serviceLog(); }
  aimConsoleService();
}
