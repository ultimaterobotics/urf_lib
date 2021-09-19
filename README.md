# urf_lib
Library with BLE stack implementation and various useful functions for nRF52832 MCU

Please note that code inside nrf_usdk52 folder is licensed under different terms (coming from Nordic and ARM, in contrary to all other code which is developed by our team) and is not subject of MIT license used for all other code

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
