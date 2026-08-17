// aim_catalog.h — AIM Network v0.6.x message catalog.
// This is the single source of truth for classes, subjects, scaling, and
// priorities on the wire. Any change here must bump kSchemaVersion in the
// same commit (mismatched schema versions are a ground-abort item).

#ifndef AIM_CATALOG_H
#define AIM_CATALOG_H

#include <cstdint>

namespace aim {

// Wire schema version, carried in every HEARTBEAT frame. Independent of the
// library semver in library.json — do not merge them.
static constexpr uint8_t kSchemaVersion = 3U;

// --- CAN ID fields (29-bit extended ID) ---
// | 28:25 class | 24:17 subject | 16:13 source | 12:0 reserved=0 |

enum class Class : uint8_t {
  Event     = 0x0,  // Highest priority (SafeStateEntered clears the bus immediately)
  Time      = 0x1,  // High priority to minimize clock synchronization jitter/error
  Cmd       = 0x2,
  Ack       = 0x3,
  State     = 0x4,
  Sensor    = 0x5,
  Heartbeat = 0x6,
  Debug     = 0x7,  // Lowest priority
};

enum class Source : uint8_t {
  // 0x0 deliberately invalid: an all-zero ID field is a wiring-fault symptom.
  Ucm       = 0x1,
  Lcm       = 0x2,
  Altimeter = 0x3,
  Gps       = 0x4,
  Comms     = 0x5,
  Power     = 0x6,
  // 0x7–0xE spare (test rig, debug dongle, future boards)
};

namespace subject {
// Valves (shared across CMD / ACK / STATE)
static constexpr uint8_t Heartbeat = 0x00;
static constexpr uint8_t Av203     = 0x01;  // LCM, CAN-commanded
static constexpr uint8_t Av205     = 0x02;  // LCM, CAN-commanded
static constexpr uint8_t Av204     = 0x03;  // UCM, Vent valve
// Power FETs
static constexpr uint8_t PwrPtUcm  = 0x05;
static constexpr uint8_t PwrSolLcm = 0x06;
static constexpr uint8_t PwrPtLcm  = 0x07;
// Sensors (wire value = i32 fixed-point)
static constexpr uint8_t Pt204        = 0x10;  // LCM, PSI (x100)
static constexpr uint8_t Pt202        = 0x11;  // UCM, PSI (x100)
static constexpr uint8_t Acceleration = 0x12;  // Altimeter, 3-axis magnitude in mm/s² (= m/s² × 1000; 1G = 9810)
static constexpr uint8_t PtSpare1     = 0x13;  // UCM, spare ADC channel, PSI (x100)
static constexpr uint8_t PtSpare2     = 0x14;  // LCM, spare ADC channel, PSI (x100)
static constexpr uint8_t TcLowerValve = 0x15;  // LCM, Celsius (x100)
static constexpr uint8_t Altitude     = 0x18;  // meters (x100)
static constexpr uint8_t GpsPosition  = 0x19;  // lon: b[0..3], lat: b[4..7] (1e-7 deg)
static constexpr uint8_t BattVolt     = 0x20;  // mV
static constexpr uint8_t GpsNumSats   = 0x21;  // count
static constexpr uint8_t Volt24Ucm    = 0x32;  // mV
static constexpr uint8_t VoltSolLcm   = 0x33;  // mV
// Events
static constexpr uint8_t LowPower          = 0x40;  // detail: 0=exit 1=enter
static constexpr uint8_t LaunchDetect      = 0x41;  // detail: 1=detected
static constexpr uint8_t GroundPowerStatus = 0x42;  // detail: 1=conected
static constexpr uint8_t SafeStateEntered  = 0x43;  // detail: reason code
static constexpr uint8_t TelemetryMode     = 0x44;  // detail: 0=Idle (1 Hz), 1=Active (100 Hz)
// Time
static constexpr uint8_t TimeSync = 0x50;
}  // namespace subject

// --- Payload enums ---

enum class FlightPhase : uint8_t {
  Preflight = 0,
  Boost     = 1,
  Coast     = 2,
  Decent    = 3,
  Landed    = 4,
};

enum class NodeState : uint8_t {
  Init      = 0,
  Nominal   = 1,
  SafeState = 2,
  Fault     = 3,
};

static inline constexpr uint16_t classBit(Class cls) {
  return static_cast<uint16_t>(1U << static_cast<uint8_t>(cls));
}

static inline constexpr bool isZeroTimestamp(Class cls, uint8_t subject) {
  return (cls == Class::Cmd) || (cls == Class::Ack) || (cls == Class::Sensor && subject == subject::GpsPosition);
}

}  // namespace aim

#endif  // AIM_CATALOG_H
