# AIM Network Protocol — v0.7.0

CAN 2.0B, 29-bit extended ID, 1 Mbps. All multi-byte fields **little-endian**
(decided: both MCUs are LE, no interop requirement with anyone else; zero byte-swapping).

**Unique-ID invariant:** every frame carries its sender's source bits, therefore no two nodes
can ever transmit the same CAN ID. Any future ID-layout change must preserve this.

---

## 1. CAN ID Layout

The 29-bit extended identifier is ordered as a flat sequence to natively dictate hardware arbitration priorities on the CAN bus:

```
| Bit 28:25 | Bit 24:17 | Bit 16:13 | Bit 12:0     |
| Class     | Subject   | Source    | Reserved = 0 |
|  4 bits   |  8 bits   |  4 bits   |  13 bits     |
```

Rules:
- **Priority is implicit in the CAN ID.** Class determines priority first, followed by Subject, and then Source.
- **Reserved bits are always 0.** Transmit 0, receivers ignore. Not allocated until a real need exists.
- **Subject identifies the data, not the route.** Same subject is reused across classes
  (e.g. subject `Av204` appears as CMD, ACK, and STATE).
- **Source is traceability, not routing.** No destination field; consumers filter by class/subject.

### Class Priority (4 bits)

| Class | Name | Direction | Description |
|---|---|---|---|
| 0x0 | EVENT | owner → all | Abort/Safety transition events (Highest Priority) |
| 0x1 | TIME | time master → all | Clock synchronization broadcasts |
| 0x2 | CMD | UCM → LCM only | Valve actuation commands |
| 0x3 | ACK | LCM → UCM only | Command receipt/result handshakes |
| 0x4 | STATE | valve owner → all | Valve physical position feedback |
| 0x5 | SENSOR | owner → all | Transducer, thermocouple, and health telemetry |
| 0x6 | HEARTBEAT | each node → all | Routine node status reports |
| 0x7 | DEBUG | any | Diagnostic output (never parsed by flight logic) |

### Source Node ID (4 bits)

| ID | Node |
|---|---|
| 0x1 | UCM |
| 0x2 | LCM |
| 0x3 | Altimeter |
| 0x4 | GPS |
| 0x5 | Comms |
| 0x6 | Power |
| 0x7–0xE | spare (test rig, debug dongle, future boards) |

*0x0 is unused (an all-zero ID field is a common wiring-fault symptom; don't make it valid).*

---

## 2. Data Field Layout (8 bytes, by class)

Standard payloads use a **16-bit millisecond offset** in bytes 0-1 (relative to the 1 Hz `TimeSync` master frame) to reclaim 2 bytes of the payload for data, leaving 6 bytes (`b[6]`) of raw payload. 

Certain subjects (such as `GpsPosition`) are designated as **Zero-Timestamp** frames via a compile-time layout trait. These frames do not carry the millisecond offset and utilize the full 8 bytes of the CAN frame payload for data (`b[8]`).

### Wire Formats

| Class | Byte 0–1 | Byte 2–7 (Data b[6]) |
|---|---|---|
| **EVENT** | offsetMs (u16) | detail (u8), 0, 0, 0, 0, 0 |
| **TIME** (Sync)* | offsetMs = 0 | timestampMs (u32), 0, 0 |
| **CMD** | offsetMs (u16) | seq (u8), desired_state (u8), 0, 0, 0, 0 |
| **ACK** | offsetMs (u16) | seq (u8), result (u8), 0, 0, 0, 0 |
| **STATE** | offsetMs (u16) | commanded (u8), energized (u8), hall (u8), 0, 0, 0 |
| **SENSOR** (Std) | offsetMs (u16) | value (i32, scaling per subject), 0, 0 |
| **SENSOR** (GPS) | GPS Lat (i32) (Byte 0–3) | GPS Lon (i32) (Byte 4–7) *(Zero-Timestamp format)* |
| **HEARTBEAT** | offsetMs (u16) | node_state (u8), error_bits (u16), schema_version (u8), 0, 0 |

*\*For TIME class / TimeSync subject, the timestampMs payload represents the master node's synchronized Unix-relative millis, baseline offset is 0.*

*   **Control state enum** (used in CMD desired, STATE commanded/hall): `0=CLOSED, 1=OPEN, 2=UNKNOWN, 3=FAULT`.
*   `energized`: `0=OFF, 1=ON`. Controls without feedback/sensing (e.g. N2 supply) report hall = UNKNOWN always.
*   **ACK result enum**: `0=ACCEPTED, 1=REJECT_SAFE_STATE, 2=REJECT_BAD_SUBJECT, 3=REJECT_BAD_STATE_VALUE`
*   **Command idempotency rule:** a CMD re-received with an already-processed seq number is re-ACKed with the original result and otherwise ignored.
*   **Node state enum**: `0=INIT, 1=NOMINAL, 2=SAFE_STATE, 3=FAULT`

---

## 3. Subject Catalog

### Controls (subjects shared by CMD / ACK / STATE)

| Subject | Name | Board | Hall? | Fail-safe bias | CAN-commanded? | STATE rate |
|---|---|---|---|---|---|---|
| 0x01 | Av203 | LCM | yes | open (N2O pressure) | **yes** | 1-2 Hz |
| 0x02 | Av205 | LCM | yes | closed | **yes** | 1-2 Hz |
| 0x03 | Av204 | UCM | yes | open (spring) | no (Wi-Fi/local) | 1-2 Hz |

### Sensors (class SENSOR, value = i32 unless noted)

| Subject | Name | Board | Units (scaling) | Rate | Prio (Jitter Protection) |
|---|---|---|---|---|---|
| 0x10 | Pt204 | LCM | PSI ×100 | 100+ Hz | High |
| 0x11 | Pt202 | UCM | PSI ×100 | 50 Hz | High |
| 0x12 | Acceleration | Altimeter | m/s² ×100 | 50 Hz | High |
| 0x13 | Velocity | Altimeter | m/s ×100 | 50 Hz | High |
| 0x15 | TcLowerValve | LCM | °C ×100 | 5 Hz | High |
| 0x18 | Altitude | Altimeter | meters ×100 | 10 Hz | High |
| 0x19 | GpsPosition | GPS | lat (i32) + lon (i32) | 5 Hz | High |
| 0x20 | BattVolt | Power | mV | 1 Hz | Medium |
| 0x21 | GpsNumSats | GPS | count | 1 Hz | Medium |
| 0x32 | Volt24Ucm | UCM | mV | 5 Hz | Low |
| 0x33 | VoltSolLcm | LCM | mV | 5 Hz | Low |

### Events (class EVENT)

Sent at Event priority class (`0x0`); repeat 3× at 100 ms spacing on state change.

| Subject | Name | Owner | detail byte |
|---|---|---|---|
| 0x40 | LowPower | Power | 0=exit, 1=enter |
| 0x41 | LaunchDetect | Power | 1=detected |
| 0x42 | SafeStateEntered | any | reason code |

### Time / Heartbeat

| Subject | Name | Notes |
|---|---|---|
| 0x50 | TimeSync | 1 Hz clock master synchronization baseline. |
| 0x00 | Heartbeat | Subject 0; source field identifies node. Sent only if node silent > T/2. |

---

## 4. Liveness & Safing
- Liveness timer per watched node resets on **any valid frame** from it.
- Timeout $\rightarrow$ de-energize all local solenoids (hardware fail-safe positions).
- Post-launch safing: LCM-local timer (4–5 min) armed by LAUNCH_DETECT; runs even if bus is dead.
- ACK means "command received and accepted" — never "valve moved." Physical confirmation is the STATE frame.

## 5. Extensibility reserves
8 spare classes · ~230 spare subjects · 8 spare source IDs · 13 reserved ID bits ·
schema_version (3U) in every heartbeat (mismatch = ground abort item).
