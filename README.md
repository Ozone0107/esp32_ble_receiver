# ESP32 BLE Synchronized Receiver

A high-precision synchronization system for ESP32 devices using Bluetooth Low Energy (BLE) broadcasting. This project allows multiple "Receiver" devices to execute actions in almost perfect sync, triggered by a central "Sender".

## Overview

This system minimize wireless latency by separating the command transmission from the execution time.

1. The Sender broadcasts a command saying: "Do X in 4000ms".

2. The Receiver gets the message, calculates the exact remaining time, and sets a hardware timer.

3. The Result: Even if devices receive the packet at slightly different times, they all trigger the action at the exact same target timestamp.

## Architecture & Logic
### How Synchronization Works

1.  **Sender (External)**
    Broadcasts a BLE Advertising packet containing a **Target Delay** and a **Command** (e.g., `PLAY`).

2.  **Fast Parsing (ISR)**
    The ESP32 receives the packet. `fast_parse_and_trigger` immediately validates the Manufacturer ID and Target Mask inside the interrupt service routine to minimize jitter.

3.  **Burst Averaging**
    To account for radio jitter, the receiver collects a burst of identical packets within a `sync_window_us` (e.g., 500ms) and averages their calculated target times.

4.  **Timer Scheduling**
    * **Action Timer:** Sets a high-resolution timer (`esp_timer`) to fire exactly when the action should happen.
    * **Prep Timer:** Optionally handles "pre-action" events (like turning off a "Ready" LED).

### Trigger States

The system exposes three trigger moments via the callback:

* `BT_TRIG_IMMEDIATE`
    Triggered immediately upon receiving valid packets (e.g., turn **ON** a "Ready" LED).
* `BT_TRIG_PREP_END`
    Triggered shortly before the main action (e.g., turn **OFF** the "Ready" LED).
* `BT_TRIG_SYNC`
    The exact synchronized moment (e.g., start music/motor).
## Getting Started

1. Two ESP32. One build and flash this project. The other one build and flash the project on https://github.com/jun16ee/ble_adv_API_test
2. Use the controller from https://github.com/jun16ee/bt_controller
3. Run the demo.