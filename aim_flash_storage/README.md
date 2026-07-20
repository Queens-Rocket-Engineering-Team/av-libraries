# aim_flash_storage

High-reliability flash storage and telemetry recording library for QRET avionics modules.

## Architecture

The library is decomposed into a backend-agnostic filesystem, a telemetry
recorder, and an optional debug console:

1.  **AimFileSystem** (`aim_file_system.h`): Owns the LittleFS instance and talks
    to a hardware driver — `ESP32PartitionDriver` (ESP32 internal-flash
    partition) or `SpiNorFlashDriver` (external SPI NOR flash; auto-detects any
    W25Q-compatible chip — e.g. W25Q128JV or BY25Q128ES — from its JEDEC ID).
2.  **AimFlightRecorder** (`aim_flight_recorder.h`): High-speed telemetry logger
    using RDES (Relative Delta Encoding System) compression, with watchdog-safe
    chunked log rotation and a streaming serial dump.
3.  **aim_console** (`aim_console.h`): Optional hook-based serial debug console.
    The library owns the flash sub-menu (info / dump / erase); applications
    register board-specific commands as a static `AimConsoleHook` array.

Node identity and the telemetry schema are build-time constants — there is no
runtime configuration store.

---

## Flight Recorder & Datalogging

High-speed datalogging with space-efficient compression.

### Logging Data
```cpp
#include <aim_file_system.h>
#include <aim_flight_recorder.h>

static const char* const kHeaders[4] = {"Time", "P1", "P2", "State"};

ESP32PartitionDriver g_hw("storage");      // or: SpiNorFlashDriver g_hw(csPin, spi);
AimFileSystem        g_fs(&g_hw);
AimFlightRecorder    g_recorder(g_fs, 4, 100, 1024 * 1024, kHeaders);
                                  // 4 cols, 100-row origin refresh, 1 MB log cap

void setup() {
    g_fs.begin();
    g_recorder.begin();
}

void loop() {
    uint32_t row[4] = { millis(), p1, p2, state };
    g_recorder.writeRow(row);
}
```

### Retrieving Data (Extract Tool)
The `extract_tool/` folder downloads and decompresses the flight log over
serial. The firmware must be running `aim_console`.

1.  Connect the board via USB-serial.
2.  Install dependencies: `pip install pyserial`
3.  Run the tool: `python extract_tool/main.py [port]` (default `/dev/ttyUSB0`)

The tool enters the console, navigates the fixed flash-dump path (`d → f → 2`),
reads the self-describing handshake (board name, column count, headers),
downloads `log.bin`, decompresses the RDES stream, and writes a timestamped CSV.

---

## Technical Standards & Safety

- **Zero Dynamic Allocation**: No `malloc` or `new` in the logging or dump paths.
- **Watchdog Safe**: Dumps and log rotation are chunked/non-blocking — each
  `serviceDump()` tick processes a bounded byte budget.
- **RDES Compression**: Efficiently encodes relative changes in sensor data to
  maximize flash lifespan.
- **No External Dependencies**: LittleFS is vendored; the SPI NOR driver speaks
  the standard W25Q command set directly (no SerialFlash library).
