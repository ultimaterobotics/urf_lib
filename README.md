# urf_lib
Library with BLE stack implementation and various useful functions for nRF52832 MCU

Please note that code inside nrf_usdk52 folder is licensed under different terms (coming from Nordic and ARM, in contrary to all other code which is developed by our team) and is not subject of MIT license used for all other code

## Used by

This library is used as a git submodule by:
- [uMyo firmware](https://github.com/ultimaterobotics/uMyo_firmware)
- [umyo-bootloader](https://github.com/ultimaterobotics/umyo-bootloader)

## Using as a submodule

```
git submodule add https://github.com/ultimaterobotics/urf_lib
git submodule update --init --recursive
```

## What it covers
Apart from BLE stack, it also implements star-type direct radio protocol, and allows somewhat simplier implementation of custom protocols. Also it covers timing and UART functions.

For using BLE, following files must be included into compilation:
urf_timer.c
urf_radio.c
urf_ble_peripheral.c
urf_ble_att_process.c
urf_ble_encryption.c
urf_ble_smp_process.c

#### urf_timer
Contains time tracking, delay, and scheduled task functions. Among other things, provides Arduino-style functions millis(), micros(), delay_ms(), delay_mcs(). Detailed description: https://github.com/ultimaterobotics/urf_lib/blob/main/urf_timer.md

#### urf_radio
Radio functions. Provides simple init functions for radio hardware (with some assumptions, but those can be overriden without side effects), handles rx/tx modes and places received packets in temporary buffer for processing outside of interrupt function (depending on the project, may be convenient or unusable). Allows setting custom interrupt processing funuction instead of default one. Detailed description: https://github.com/ultimaterobotics/urf_lib/blob/main/urf_radio.md

#### urf_ble_peripheral
BLE functions. Has simple interface for sending advertising packets, manages BLE connection state, service discovery, and allows reading/writing BLE characteristics. Calls GATT related functions from **urf_ble_att_process.c**, SMP related functions from **urf_ble_smp_process.c** and if encryption is used, hardware encryption is accessed from **urf_ble_encryption.c**. Detailed description: https://github.com/ultimaterobotics/urf_lib/blob/main/urf_ble_peripheral.md

#### urf_star_protocol
Implements a star topology radio protocol where one central node communicates with multiple peripheral nodes over a shared channel using time-division multiplexing. Each node is assigned a time slot (phase) within a repeating cycle. The central sends data to individual nodes by node ID; nodes queue packets for transmission to the central. Processing is driven by a `star_loop_step()` call that must be invoked regularly (every millisecond or so) from the main loop rather than from a timed interrupt, keeping radio switching and data copying in a short ISR and longer processing out of interrupt context. Also provides a synchronized time reference (`star_get_synced_time()`) across the network.

#### urf_uart
UART functions. Initializes UART on configurable TX/RX pins at a given baud rate. Provides non-blocking send with a `uart_send_remains()` query to check completion, and a ring buffer for received data accessible via `uart_get_rx_buf()` and `uart_get_rx_position()`. Also provides `uprintf()` for formatted debug output (printf-style) and `uhex_print()` for hex dumps.

## Known critical configuration

**`BLE_FORCE_SAME_EVENT_RESPONSE` must be `0` for Android GATT connections to work.**

When this macro is non-zero, the device responds to a BLE connection event in the same radio event it was received, which causes Android to disconnect immediately after the initial connection is established. Android's BLE stack does not tolerate this timing. Do not set this to 1 in any project that needs to connect to Android devices.

The macro can be overridden per-project without modifying the library: define `BLE_FORCE_SAME_EVENT_RESPONSE 0` in your project before including the library headers.
