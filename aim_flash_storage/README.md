# aim_flash_storage

High-reliability flash telemetry recording and serial extraction library for QRET avionics modules (STM32 & ESP32).

---

## 3-Step Quick Start: Using Flash Effectively

### Step 1: Initialize Filesystem & Flight Recorder (Firmware Setup)

```cpp
#include <aim_file_system.h>
#include <aim_flight_recorder.h>
#include <aim_console.h>

static const char* const kHeaders[4] = {"Time_ms", "Pressure_Pa", "AccelZ_mg", "State"};

// STM32 SPI NOR Flash Driver (or ESP32PartitionDriver g_hw("storage") for ESP32)
SpiNorFlashDriver g_hw(pins::kFlashCs, SPI2);
AimFileSystem     g_fs(&g_hw);
AimFlightRecorder g_recorder(g_fs, 4, 100, 1024 * 1024, kHeaders);

void setup() {
    Serial.begin(115200);
    g_fs.begin();       // Mounts LittleFS (auto-formats blank flash on first boot)
    g_recorder.begin(); // Opens active flight log (/flight/log_XXX.bin)
    aimConsoleInit(Serial, g_fs, g_recorder, "Altimeter_Module", nullptr, 0);
}
```

### Step 2: Log Telemetry Rows in Loop

```cpp
void loop() {
    if (!aimConsoleIsActive()) {
        uint32_t row[4] = { millis(), currentPressure, accelZ, flightState };
        g_recorder.writeRow(row); // RDES-compresses & logs row (~20B/row)
    }
    aimConsoleService(); // Services serial extraction requests
}
```

### Step 3: Extract Log to CSV Over Serial

Run the Python extraction script on your ground PC:

```powershell
python av-libraries/aim_flash_storage/extract_tool/main.py COM3
```
*Navigates serial console menu (`q -> d -> f -> 2`), receives `#` handshake, downloads 512B blocks, and decompresses into `{board}_{log}_{timestamp}.csv`.*

---

## Essential Hardware Traps & Checkpoints

- **Serial Baud Rate**: Python tool defaults to **115,200 baud**. Ensure firmware `Serial.begin(115200)` matches (or use `--baud`).
- **CS Pin High**: SPI Flash Chip Select (e.g. `PB12`) must be set `OUTPUT` and `HIGH` before `g_fs.begin()`.
- **Watchdog Timeout**: Physical 4 KB sector erases take **40–100 ms**. Keep hardware watchdog $\ge 1.0\text{ s}$ to prevent resets.
