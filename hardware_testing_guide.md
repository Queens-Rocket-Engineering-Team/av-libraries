# Hardware Testing Guide & Risk Checklist: `aim_flash_storage` & RDES

This pre-flight hardware testing guide details the top 5 physical failure points, pre-test verification steps, log extraction procedures, and flash logging job rate dynamics for testing `aim_flash_storage` and `aim_rdes` on real vehicle node hardware (STM32 / ESP32).

---

## 1. Top 5 Real-World Hardware Failure Points

### 1. Serial Baud Rate Mismatch on Log Extraction
* **Code Location**: [`av-power/Firmware/Hybrid_Power_Module/src/main.cpp:64`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-power/Firmware/Hybrid_Power_Module/src/main.cpp#L64) vs [`extract_tool/main.py:23`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-libraries/aim_flash_storage/extract_tool/main.py#L23).
* **Risk**: `Hybrid_Power_Module` initializes `SoftwareSerial` at **38,400 baud**, while `extract_tool/main.py` defaults to **115,200 baud**.
* **Symptom**: `extract_tool/main.py` fails to connect or times out waiting for the dump handshake: `IOError("No dump handshake ('#')")`.
* **Fix**: Ensure serial baud rates match between Python tool and target firmware (or update firmware to use hardware UART at 115,200 baud).

---

### 2. SPI Flash JEDEC ID Failure (`SpiNorFlash: no device`)
* **Code Location**: [`SpiNorFlashDriver::begin()`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-libraries/aim_flash_storage/src/aim_file_system.h#L77-L83).
* **Risk**: `SpiNorFlashDriver` sends JEDEC ID read command `0x9F` over SPI and expects 3 bytes `[Manufacturer ID, Memory Type, Capacity]`. If `id[0] == 0xFF` (CS pin floating or MOSI disconnected) or `0x00` (MISO tied low), initialization fails.
* **Symptom**: Serial monitor prints `LOG_ERROR("SpiNorFlash: no device")` and logging is disabled.
* **Fix**:
  - Verify Chip Select (CS) pin assignment in `pinouts.h` (`pins::kFlashCs`, e.g. `PB12`).
  - Confirm `pinMode(csPin, OUTPUT)` and `digitalWrite(csPin, HIGH)` execute before `g_fs.begin()`.

---

### 3. SPI Bus Contention with Sensors (IMU / Barometer)
* **Code Location**: [`SpiNorFlashDriver::read` and `prog`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-libraries/aim_flash_storage/src/aim_file_system.h#L90-L117).
* **Risk**: `SpiNorFlashDriver` transfers bytes directly via `_spi.transfer()` without wrapping calls in `_spi.beginTransaction()` / `_spi.endTransaction()`.
* **Symptom**: If an IMU or Barometer on `SPI2` is queried via interrupts while `SpiNorFlashDriver` is in a 256-byte page program loop, SPI signals will clash, causing `LOG_ERROR("SpiNorFlash: busy timeout")` or corrupted sensor telemetry.
* **Fix**: Dedicate SPI bus to flash (e.g. `SPI2` for flash, `I2C`/`SPI1` for sensors) or wrap SPI calls in RTOS bus lock mutexes.

---

### 4. Main Loop Jitter During Sector Erases (40 ms – 100 ms)
* **Code Location**: [`SpiNorFlashDriver::erase`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-libraries/aim_flash_storage/src/aim_file_system.h#L119-L124).
* **Risk**: Physical Winbond W25Q128 / BY25Q128 flash chips take **40 to 100 milliseconds** to execute a 4 KB sector erase (`0x20`).
* **Symptom**: Every time LittleFS fills a 4 KB sector and allocates a new block, `writeRow()` will block for ~40–100 ms while polling status register `0x05`.
* **Impact**: Normal physical hardware behavior. Main loop timing will show a brief ~50 ms latency jump every ~4 KB of logged data. Watchdog timeout (`kWatchdogTimeoutUs = 2000000U` = 2.0s) is safe and will not trigger.

---

### 5. First-Boot Auto-Format Pause (500 ms)
* **Code Location**: [`AimFileSystem::begin()`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-libraries/aim_flash_storage/src/aim_file_system.cpp#L40-L47).
* **Risk**: Brand-new SPI NOR flash chips from the factory come with all bytes set to `0xFF` (unformatted).
* **Symptom**: On the **very first boot** of a fresh board, `lfs_mount` fails with `LFS_ERR_CORRUPT` (-84). `AimFileSystem` logs `LOG_WARN("Flash mount failed (err=-84), attempting format")` and takes ~500 ms to format the chip.
* **Impact**: Expected behavior. On all subsequent boots, `lfs_mount` succeeds in <1 ms.

---

## 2. Pre-Hardware Test Checklist

| Step | Verification Item | Target File / Location | Check Method |
| :--- | :--- | :--- | :--- |
| **[ ] 1** | **Serial Baud Rate Matching** | `extract_tool/main.py` vs node `main.cpp` | Verify baud rates match prior to running Python extraction tool. |
| **[ ] 2** | **Flash CS Pin High** | `pinouts.h` / node `main.cpp` | Confirm CS pin (e.g. `PB12`) is configured as `OUTPUT` and set `HIGH`. |
| **[ ] 3** | **3.3V Power Rail Capacity** | Hardware Board Schematics | Ensure 3.3V regulator can deliver $\ge 30\text{ mA}$ peak current during SPI page programming. |
| **[ ] 4** | **Watchdog Timeout Margin** | Node `main.cpp` (`kWatchdogTimeoutUs`) | Confirm watchdog timeout is set to $\ge 1.0\text{ s}$ to accommodate sector erases. |
| **[ ] 5** | **First Boot Log Output** | Serial Monitor @ 115200 / 38400 | Verify serial log prints `Flash mounted: N blocks of 4096 bytes` or `Flash ready`. |

---

## 3. Flash Logging Job Rates & Timing Analysis

### Dynamic Rate Shifting Architecture
Vehicle nodes operate under a dual-rate job schedule ([`Altimeter_Module/src/node.cpp:34-35`](file:///C:/Users/tristan/Documents/qret/AimNetwork/av-recovery/Firmware/Altimeter_Module/src/node.cpp#L34-L35)):
* **Pad Idle Rate (1 Hz / 1000 ms)**: Writes 1 row/sec on the launch pad (~20 B/s), minimizing flash writes and CPU load while waiting for launch.
* **Active Flight Rate (100 Hz / 10 ms)**: Triggered dynamically over CAN upon `aim::subject::LaunchDetect` or `TelemetryMode` event. Captures high-g and barometric data with 10 ms temporal resolution.

### Physical SPI Bandwidth at 100 Hz
* **RDES Compressed Row Size**: ~20 Bytes/row across 10 telemetry columns.
* **Flash Bandwidth Demand**: $20\text{ B/row} \times 100\text{ rows/sec} = \mathbf{2.0\text{ KB/second}}$.
* **Hardware SPI Transfer Time**: $< 10\text{ }\mu\text{s}$ per row over 8 MHz SPI bus.
* **CPU Overhead**: $< 0.5\%$ CPU usage on 72 MHz ARM Cortex-M3.

### Sensor Sampling vs 10 ms Budget
* Reading MS5611 Baro (400 kHz I2C): **~1.5 ms**
* Reading MPU6050 IMU (400 kHz I2C): **~1.2 ms**
* Reading KX134 Accel (400 kHz I2C): **~0.8 ms**
* **Total Sensor Read Time**: **~3.5 ms** per iteration.
* **Idle CPU Margin**: **6.5 ms remaining per 10 ms frame (65% idle headroom)**.

---

## 4. Post-Flight / Bench Log Extraction Procedure

### Step 1: Connect Serial Interface
Connect FTDI / USB-to-Serial adapter to the target board's debug console serial pins.

### Step 2: Run Python Ground Extraction Tool
Execute `extract_tool/main.py` from your terminal:
```powershell
python av-libraries/aim_flash_storage/extract_tool/main.py COM3
```
*(Replace `COM3` with your actual serial port, e.g. `/dev/ttyUSB0` on Linux).*

### Step 3: Verify Output CSV
The extraction script will:
1. Issue menu shortcut sequence (`q -> d -> f -> 2`).
2. Receive binary handshake packet (`#` + block count + board name + column schema).
3. Display storage capacity and download 512-byte payload blocks.
4. Invoke `RDESDecompressorCTypes` (using `librdes.dll`) to decompress telemetry deltas.
5. Export a timestamped CSV file: `{board_name}_{log_name}_{YYYYMMDD_HHMMSS}.csv`.
