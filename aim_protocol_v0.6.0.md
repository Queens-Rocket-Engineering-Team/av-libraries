# AIM Network Protocol — v0.6.0 DRAFT (for review)

CAN 2.0B, 29-bit extended ID, 500 kbps. All multi-byte fields **little-endian**
(decided: both MCUs are LE, no interop requirement with anyone else; zero byte-swapping).

**Unique-ID invariant:** every frame carries its sender's source bits, therefore no two nodes
can ever transmit the same CAN ID. Any future ID-layout change must preserve this.

Items marked **[?]** need confirmation.

---

## 1. CAN ID Layout

```
| Bit 28:27 | Bit 26:23 | Bit 22:15 | Bit 14:11 | Bit 10:0     |
| Priority  | Class     | Subject   | Source    | Reserved = 0 |
|  2 bits   |  4 bits   |  8 bits   |  4 bits   |  11 bits     |
```

Rules:

- **Priority is fixed per message definition** (column in the catalog), never chosen at runtime.
- **Reserved bits are always 0.** Transmit 0, receivers ignore. Not allocated until a real need exists.
- **Subject identifies the data, not the route.** Same subject is reused across classes
  (e.g. subject `VALVE_4` appears as CMD, ACK, and STATE).
- **Source is traceability, not routing.** No destination field; consumers filter by class/subject.

### Priority levels (2 bits)

| Value | Meaning | Traffic |
|---|---|---|
| 0 | Safety / control | SAFE_STATE event, valve CMD + ACK |
| 1 | Coordination | TIME sync, valve STATE |
| 2 | Fast telemetry | PTs |
| 3 | Background | TC, voltages, GPS, altitude, heartbeat, events, debug |

### Class (4 bits) — 8 defined, 8 spare

| Class | Name | Direction |
|---|---|---|
| 0x0 | CMD | UCM → LCM only |
| 0x1 | ACK | LCM → UCM only |
| 0x2 | STATE | valve owner → all |
| 0x3 | SENSOR | owner → all |
| 0x4 | TIME | time master → all |
| 0x5 | HEARTBEAT | each node → all |
| 0x6 | EVENT | owner → all |
| 0x7 | DEBUG | any (never parsed by flight logic) |

### Source (4 bits)

| ID | Node |
|---|---|
| 0x1 | Comms |
| 0x2 | UCM |
| 0x3 | LCM |
| 0x4 | Altimeter |
| 0x5 | GPS |
| 0x6 | Power |
| 0x7–0xE | spare (test rig, debug dongle, future boards) |

0x0 unused (an all-zero ID field is a common wiring-fault symptom; don't make it valid).

---

## 2. Data Field Layout (8 bytes, by class)

All frames: **bytes 0–3 = timestamp, ms since start of day (uint32)**, from sender's synced clock.

| Class | Bytes 4–7 |
|---|---|
| CMD | seq (u8), desired_state (u8), 0, 0 |
| ACK | seq (u8), result (u8), 0, 0 |
| STATE | commanded (u8), energized (u8), hall (u8), 0 |
| SENSOR | value (i32, scaling per subject) |
| TIME | flags (u8: bit0 = GPS-disciplined), 0, 0, 0 — timestamp field IS the payload |
| HEARTBEAT | node_state (u8), error_bits (u16), schema_version (u8) |
| EVENT | detail (u8, per subject), 0, 0, 0 |

Control state enum (used in CMD desired, STATE commanded/hall): `0=CLOSED, 1=OPEN, 2=UNKNOWN, 3=FAULT`.
`energized`: `0=OFF, 1=ON`. Controls without feedback/sensing (e.g. N2 supply) report hall = UNKNOWN always.

ACK result enum: `0=ACCEPTED, 1=REJECT_SAFE_STATE, 2=REJECT_BAD_SUBJECT, 3=REJECT_BAD_STATE_VALUE`
(spare values reserved for faults discovered in testing).

**Command idempotency rule:** a CMD re-received with an already-processed seq number is re-ACKed
with the original result and otherwise ignored. UCM retry = timeout + resend, no handshake needed.

Node state enum: `0=INIT, 1=NOMINAL, 2=SAFE_STATE, 3=FAULT`

---

## 3. Subject Catalog

### Controls (subjects shared by CMD / ACK / STATE)

| Subject | Name | Board | Hall? | Fail-safe bias | CAN-commanded? | STATE rate |
|---|---|---|---|---|---|---|
| 0x01 | VALVE_1 | UCM | yes | open (spring) | no (Wi-Fi/local) | 1-2 Hz |
| 0x02 | VALVE_2 | UCM | no (COTS) | open (spring) | no (Wi-Fi/local) | 1-2 Hz |
| 0x03 | VALVE_3 | LCM | yes | open (N2O pressure) | **yes** | 1-2 Hz |
| 0x04 | VALVE_4 | LCM | yes | closed | **yes** | 1-2 Hz |

*UCM publishes STATE for its own valves so Comms/LoRa sees all four through one mechanism.*

### Sensors (class SENSOR, value = i32)

| Subject | Name | Board | Units (scaling) | Rate | Prio |
|---|---|---|---|---|---|
| 0x10 | PT_1 | UCM | PSI ×100 | 50 Hz | 2 |
| 0x11 | PT_2 | UCM | PSI ×100 | 50 Hz | 2 |
| 0x12 | PT_3 | LCM | PSI ×100 | 50 Hz | 2 |
| 0x13 | PT_4 | LCM | PSI ×100 | 50 Hz | 2 |
| 0x18 | TC_CHAMBER | LCM | °C ×100 | 5 Hz | 3 |
| 0x20 | SOLENOID_VOLT_UCM | UCM | mV | 5 Hz | 3 |
| 0x21 | SOLENOID_VOLT_LCM | LCM | mV | 5 Hz | 3 |
| 0x28 | BATT_VOLT | Power | mV | 1 Hz | 3 |
| 0x30 | GPS_LAT | GPS | degrees ×10⁷ | 5 Hz | 3 |
| 0x31 | GPS_LON | GPS | degrees ×10⁷ | 5 Hz | 3 |
| 0x32 | GPS_NUM_SATS | GPS | count | 1 Hz | 3 |
| 0x38 | ALTITUDE | Altimeter | meters ×100 | 10 Hz | 3 |

All sensor values are scaled signed integers — no floats on the wire. Scaling lives in
this table and the generated header, nowhere else.

### Events (class EVENT, sent at prio 0 or 3 as listed; repeat 3× at 100 ms spacing — events are state changes, not single frames)

| Subject | Name | Owner | detail byte | Prio |
|---|---|---|---|---|
| 0x40 | LOW_POWER | Power | 0=exit, 1=enter | 3 |
| 0x41 | LAUNCH_DETECT | Power | 1=detected | 3 |
| 0x42 | SAFE_STATE_ENTERED | any | reason code | 0 |

### Time / Heartbeat

| Subject | Name | Notes |
|---|---|---|
| 0x50 | TIME_SYNC | 1 Hz (50 ppm crystal drift ⇒ sub-ms skew). Master = UCM pre-launch, GPS post-launch (flag bit0 says which). |
| 0x00 | HEARTBEAT | Subject 0; source field identifies node. Sent only if node silent > T/2. |

---

## 4. Liveness & Safing (summary of agreed behavior)

- Liveness timer per watched node resets on **any valid frame** from it.
- LCM watches UCM; UCM watches LCM; Comms watches everyone (telemetry health only).
- Timeout → de-energize all local solenoids (hardware fail-safe positions). Timeout values TBD — consider different pad vs. flight values.
- Post-launch safing: LCM-local timer (4–5 min) armed by LAUNCH_DETECT; runs even if bus is dead.
- ACK means "command received and accepted" — never "valve moved." Physical confirmation is the
  STATE frame (commanded / energized / hall), judged by humans at the GUI.

## 5. Extensibility reserves (deliberate, do not fill without need)

8 spare classes · ~230 spare subjects · 8 spare source IDs · 11 reserved ID bits ·
schema_version in every heartbeat (bump on any catalog change; mismatch = ground abort item).

## Open items

1. PT names/locations ×4; all sensor rates
2. Heartbeat/liveness timeout values, pad vs flight
3. Who owns LAUNCH_DETECT (Power physical vs Altimeter accel)
4. Midnight rollover policy: clock steps backward at 00:00; post-processing must detect the
   negative jump. Accepted quirk — written here so it's never a 2 a.m. mystery.
