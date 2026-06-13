# QRET Avionics Libraries

Shared firmware libraries for QRET's avionics stack. 

These libraries support our STM32 nodes and the ESP32 comms board. They handle the underlying communication, logging, and storage infrastructure, allowing you to focus on application logic like reading sensors and controlling actuators.

## What's Inside

- **aim_network**: Manages CAN bus communication, time synchronization, and network health monitoring seamlessly in the background.
- **aim_logger**: A lightweight logger that automatically adds timestamps and severity levels (like `DEBUG` or `ERROR`) to your messages.
- **aim_flash_storage**: Stores data directly to the board's serial flash memory or internal flash using littlefs. Includes RDES compression for telemetry.

## Setup & Build

We use **PlatformIO** to manage libraries and board configurations. 

**Installation:**
You can install the [PlatformIO IDE extension for VSCode](https://platformio.org/install/ide?install=vscode) or the standalone [PlatformIO Core CLI](https://docs.platformio.org/en/latest/core/installation.html). The official PlatformIO documentation provides excellent setup guides to get you running.

**Essential Commands:**
Whether you use the VSCode UI or the terminal, the core operations are the same. Here is how you run them from the command line:
- **Compile code:** `pio run`
- **Upload to board:** `pio run --target upload`
- **Open Serial Monitor:** `pio device monitor`

*(Tip: In VSCode, you can also use the checkmark, right arrow, and plug icons on the bottom toolbar).*

## Quick Start

1. Open an example folder (e.g., `aim_network/examples/stm32_canbus`) as a project in PlatformIO.
2. Use standard Arduino functions in the main loop to handle your hardware:
   ```cpp
   int pressureValue = analogRead(A0); 
   if (pressureValue > 500) {
       digitalWrite(LED_PIN, HIGH);
   }
   ```
3. Use the logger to track events instead of `Serial.print()`:
   ```cpp
   LOG_INFO("Pressure value is: %d", pressureValue);
   ```
4. Compile and upload your code. The provided example projects are the recommended starting points for any new firmware.

## Design Notes

- The CAN network design is intentionally narrow and predictable: mailbox-driven TX, content-based addressing (consumers filter by message class/subject — there is no destination field), bounded RX polling, and fixed-size static queues.
- STM32 uses an internal HAL-based CAN backend instead of external libraries for precise hardware control.
- ESP32 uses TWAI through the existing backend.
