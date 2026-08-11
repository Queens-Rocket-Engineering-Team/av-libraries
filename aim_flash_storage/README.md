# aim_flash_storage

High-reliability flash telemetry recording, LittleFS storage management, and serial data extraction for QRET avionics modules (STM32 & ESP32).

---

## 1. Dependencies & Implementation Truth

### Dependencies
- **Firmware**: Requires [`aim_rdes`](../aim_rdes/README.md) (uses C API [`src/rdes.h`](../aim_rdes/src/rdes.h) for telemetry compression) and [`aim_logger`](../aim_logger). Uses vendored LittleFS (`src/littlefs/`).
- **Ground Tool**: Requires `librdes.dll` (Windows) / `librdes.so` (Linux/macOS) compiled from [`aim_rdes/src/rdes.c`](../aim_rdes/src/rdes.c), and Python `pyserial`.

### Source of Implementation Truth (Working Projects)
- **STM32 (SPI NOR Flash)**: [`examples/stm32_flash/src/main.cpp`](examples/stm32_flash/src/main.cpp) — Uses `SpiNorFlashDriver` over SPI2 (`PB12` CS).
- **ESP32 (Internal Partition)**: [`examples/esp32_flash/src/main.cpp`](examples/esp32_flash/src/main.cpp) — Uses `ESP32PartitionDriver` with custom `partitions.csv`.

---

## 2. End-to-End Operational Workflow

1. **Build & Upload Firmware**:
   ```bash
   cd av-libraries/aim_flash_storage/examples/stm32_flash
   pio run --target upload
   ```

2. **First-Boot Format & Manual Erase**:
   - *First-Boot Format*: Brand-new flash chips with unformatted `0xFF` bytes auto-format LittleFS in ~500 ms on first boot.
   - *Manual Console Wipe*: Connect serial monitor (`pio device monitor`), press `d` (debug console) $\rightarrow$ `f` (flash menu) $\rightarrow$ `3` (erase) $\rightarrow$ `1` (confirm).

3. **High-Speed Telemetry Logging**:
   Gate logging in `main.cpp` so telemetry pauses while debug console is active:
   ```cpp
   void loop() {
     if (!aimConsoleIsActive()) {
       uint32_t row[4] = { millis(), p_pa, a_z, state };
       g_recorder.writeRow(row); // RDES-compresses & logs row to /flight/log_XXX.bin
     }
     aimConsoleService();
   }
   ```

4. **Ground Extraction to CSV**:
   Compile `librdes.dll` in `aim_rdes`, then run:
   ```powershell
   python av-libraries/aim_flash_storage/extract_tool/main.py COM3
   ```
   *Navigates `aim_console` (`q -> d -> f -> 2`), receives `#` binary handshake, streams 512B payload blocks, and decompresses into `{board}_{log}_{timestamp}.csv`.*

---

## 3. Console Architecture & Telemetry Gating (`aim_console`)

`aim_console` ([`src/aim_console.h`](src/aim_console.h)) provides a non-blocking serial debug UI owned by the library.

### Flash Menu Commands (`DBG > FLS`)
- **`1` (Info)**: Prints ready status, total capacity, and used bytes.
- **`2` (Dump)**: Streams binary dump of latest log file over serial. Mutes `LOG_*` printing to prevent byte corruption.
- **`3` (Erase)**: Enters confirmation menu (`1` confirms erase/format).
- **`4` (List)**: Lists all stored flight log files and sizes.
- **`i` (Index)**: Expects raw index byte `N` to stream specific log `/flight/log_00N.bin`.

### Telemetry Gating
`aimConsoleIsActive()` returns `true` whenever an operator or ground script is inside a console menu. Telemetry logging **must** be gated with `if (!aimConsoleIsActive())` to prevent file lock contention during serial dumps or formatting.

---

## 4. Hardware Edge Cases

- **Sector Erase Pauses**: SPI NOR 4 KB sector erases take **40–100 ms**. Keep hardware watchdog $\ge 1.0\text{ s}$.
- **Baud Rate Mismatch**: Python ground tool defaults to **115,200 baud**. Match firmware `Serial.begin(115200)` or pass `--baud`.
- **SPI CS Pin High**: Flash Chip Select (e.g. `PB12`) must be set `OUTPUT` and driven `HIGH` before `g_fs.begin()`.
