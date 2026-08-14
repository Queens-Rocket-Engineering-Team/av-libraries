# QRET Avionics Libraries

Shared firmware libraries for QRET's avionics stack — the communication, logging, and
storage infrastructure the STM32 and ESP32 nodes build on.

## What's Inside

- [**aim_network**](aim_network): CAN bus communication, time synchronization, and network health monitoring.
- [**aim_logger**](aim_logger): Lightweight logger that stamps each message with a timestamp and severity (`DEBUG`–`ERROR`).
- [**aim_flash_storage**](aim_flash_storage/README.md): Telemetry storage to internal or SPI NOR flash via LittleFS, serial extraction protocol, and hardware safety guidelines.
- [**aim_rdes**](aim_rdes/README.md): Realtime Deviation Encoding Scheme (RDES3) telemetry compression algorithm, C/C++ static inline library, and Python FFI bindings.

## Documentation Links

- [aim_flash_storage Documentation](aim_flash_storage/README.md)
- [aim_rdes Protocol Specification](aim_rdes/README.md)

## Build

PlatformIO manages libraries and board configs ([install guide](https://platformio.org/install)):

- Compile: `pio run`
- Upload: `pio run --target upload`
- Serial monitor: `pio device monitor`

## Quick Start

Open an example as a PlatformIO project (e.g. `aim_network/examples/stm32_canbus`) — these are the
recommended starting point for new firmware. Use standard Arduino calls for hardware, and the logger
instead of `Serial.print()`:

```cpp
int pressureValue = analogRead(A0);
LOG_INFO("Pressure value is: %d", pressureValue);
```

## Design Notes

- The CAN network is intentionally narrow and predictable: mailbox-driven TX, content-based
  addressing (consumers filter by message class/subject — there is no destination field), bounded
  RX polling, and fixed-size static queues.
- STM32 uses an internal HAL-based bxCAN backend; ESP32 uses TWAI. No external CAN libraries.

