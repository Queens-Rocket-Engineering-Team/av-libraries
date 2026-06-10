# AimFlashStorage

High-reliability flash storage and telemetry recording library for QRET avionics modules.

## Architecture

The library is decomposed into three main components:
1.  **AimFileSystem**: Owns the LittleFS instance and interfaces with Hardware Drivers (`ESP32PartitionDriver` or `SerialFlashDriver`).
2.  **AimConfigStore**: Handles atomic JSON configuration storage using `ArduinoJson`.
3.  **AimFlightRecorder**: High-speed telemetry logger using RDES (Relative Delta Encoding System).

---

## 1. JSON Configuration Store

Used for mission-critical data that must persist across reboots (calibration, network ID, flight state). Uses an atomic "write-to-tmp-then-rename" strategy to prevent corruption during power loss.

### Usage Example

```cpp
#include <AimFileSystem.h>
#include <AimConfigStore.h>

struct MyConfig {
    float pressureOffset = 0.0f;
    int nodeId = 1;
};

ESP32PartitionDriver g_hw("storage");
AimFileSystem g_fs(&g_hw);
AimConfigStore g_store(g_fs);
MyConfig g_cfg;

void setup() {
    if (g_fs.begin()) {
        JsonDocument doc;
        if (g_store.load("/config.json", doc)) {
            g_cfg.pressureOffset = doc["pressureOffset"] | 0.0f;
            g_cfg.nodeId = doc["nodeId"] | 1;
        }
    }
}

void updateCalibration(float newOffset) {
    g_cfg.pressureOffset = newOffset;
    JsonDocument doc;
    doc["pressureOffset"] = g_cfg.pressureOffset;
    doc["nodeId"] = g_cfg.nodeId;
    g_store.save("/config.json", doc); // Atomic save
}
```

---

## 2. Flight Recorder & Datalogging

High-speed datalogging with space-efficient compression.

### Logging Data
```cpp
AimFlightRecorder g_recorder(g_fs, 8, 100, 1024 * 1024); // 8 cols, 100 origin interval, 1MB limit

void loop() {
    uint32_t row[8] = { millis(), p1, p2, ... };
    g_recorder.writeRow(row);
}
```

### Retrieving Data (Extract Tool)
The `extract_tool` folder contains Python scripts for downloading and decompressing the flight log.

1.  Connect the board via USB-Serial.
2.  Install dependencies: `pip install pyserial`
3.  Run the tool: `python extract_tool/main.py`
4.  The tool will connect, perform a binary handshake, and download `log.bin`.
5.  It automatically decompresses the RDES data and saves it as a CSV.

---

## Technical Standards & Safety

- **Zero Dynamic Allocation**: No `malloc` or `new` after `setup()` (Note: `ArduinoJson` 7 `JsonDocument` uses heap, consider pre-allocating if critical).
- **Watchdog Safe**: Dumps and flash operations are designed to be non-blocking or chunked.
- **Atomic Renames**: Configuration updates never overwrite the active file directly.
- **RDES Compression**: Efficiently handles relative changes in sensor data to maximize flash lifespan.