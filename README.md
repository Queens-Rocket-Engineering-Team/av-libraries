# Avionics Libraries

This repository is work space for QRET's 2025-2026 shared Avionics firmware libraries.

## AimNetwork CAN backend notes

- STM32 now uses an in-repo HAL-based CAN backend (`AimStm32CanCore`) instead of the external `STM32_CAN` library.
- Design scope is intentionally narrow: mailbox-driven TX, destination filter setup, bounded RX polling, and fixed-size static TX/RX queues.
- ESP32 keeps using TWAI via the existing backend.
