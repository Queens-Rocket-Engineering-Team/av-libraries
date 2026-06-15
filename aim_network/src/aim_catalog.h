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
static constexpr uint8_t kSchemaVersion = 2U;

// --- CAN ID fields (29-bit extended ID) ---
// | 28:27 prio | 26:23 class | 22:15 subject | 14:11 source | 10:0 reserved=0 |

enum class Class : uint8_t {
  Cmd       = 0x0,  // UCM → LCM only
  Ack       = 0x1,  // LCM → UCM only
  State     = 0x2,  // valve owner → all
  Sensor    = 0x3,  // owner → all
  Time      = 0x4,  // time master → all
  Heartbeat = 0x5,  // each node → all
  Event     = 0x6,  // owner → all
  Debug     = 0x7,  // any; never parsed by flight logic
};

enum class Source : uint8_t {
  // 0x0 deliberately invalid: an all-zero ID field is a wiring-fault symptom.
  Comms     = 0x1,
  Ucm       = 0x2,
  Lcm       = 0x3,
  Altimeter = 0x4,
  Gps       = 0x5,
  Power     = 0x6,
  // 0x7–0xE spare (test rig, debug dongle, future boards)
};

namespace subject {
// Valves (shared across CMD / ACK / STATE)
static constexpr uint8_t Heartbeat = 0x00;
static constexpr uint8_t Av204     = 0x01;  // UCM, Vent valve
static constexpr uint8_t AvSpare   = 0x02;  // UCM, Ground side valve now DON'T ENERGIZE
static constexpr uint8_t Av203     = 0x03;  // LCM, CAN-commanded
static constexpr uint8_t Av205     = 0x04;  // LCM, CAN-commanded
// Power FETs
static constexpr uint8_t PwrPtUcm  = 0x05;
static constexpr uint8_t PwrSolLcm = 0x06;
static constexpr uint8_t PwrPtLcm  = 0x07;
// Sensors (value = i32, scaling fixed here)
static constexpr uint8_t Pt202        = 0x10;  // UCM, PSI x100
static constexpr uint8_t PtSpare1     = 0x11;  // UCM, PSI x100, no longer needed
static constexpr uint8_t Pt204        = 0x12;  // LCM, PSI x100
static constexpr uint8_t PtSpare2     = 0x13;  // LCM, PSI x100, no longer needed
static constexpr uint8_t TcLowerValve = 0x18;  // LCM, Celsius x100
static constexpr uint8_t Volt24Ucm    = 0x20;  // mV
static constexpr uint8_t VoltSolLcm   = 0x21;  // mV
static constexpr uint8_t BattVolt     = 0x28;  // mV
static constexpr uint8_t GpsLat       = 0x30;  // degrees x10^7
static constexpr uint8_t GpsLon       = 0x31;  // degrees x10^7
static constexpr uint8_t GpsNumSats   = 0x32;  // count
static constexpr uint8_t Altitude     = 0x38;  // meters x100
// Events
static constexpr uint8_t LowPower         = 0x40;  // detail: 0=exit 1=enter
static constexpr uint8_t LaunchDetect     = 0x41;  // detail: 1=detected
static constexpr uint8_t SafeStateEntered = 0x42;  // detail: reason code
// Time
static constexpr uint8_t TimeSync = 0x50;
}  // namespace subject

// Priority is fixed per message definition — never chosen by callers.
// 0 = safety/control, 1 = coordination, 2 = fast telemetry, 3 = background.
static inline uint8_t priorityFor(Class cls, uint8_t subj) {
  switch (cls) {
    case Class::Cmd:
    case Class::Ack:
      return 0U;
    case Class::Event:
      return (subj == subject::SafeStateEntered) ? 0U : 3U;
    case Class::Time:
    case Class::State:
      return 1U;
    case Class::Sensor:
      // INFO: higher priority for LCM sensors
      return (subj >= subject::Pt204) ? 2U : 3U;
    default:
      return 3U;
  }
}

// --- Payload enums (bytes 4–7, layouts per class in the protocol doc) ---

enum class ValveState : uint8_t {
  Closed  = 0,
  Open    = 1,
  Unknown = 2,  // valves without hall sensing always report Unknown
  Fault   = 3,
};

enum class AckResult : uint8_t {
  Accepted            = 0,
  RejectSafeState     = 1,
  RejectBadSubject    = 2,
  RejectBadStateValue = 3,
};

enum class NodeState : uint8_t {
  Init      = 0,
  Nominal   = 1,
  SafeState = 2,
  Fault     = 3,
};

}  // namespace aim

#endif  // AIM_CATALOG_H
