# ESP32 BLE Synchronized Receiver

A high-precision synchronization system for ESP32 devices using Bluetooth Low Energy (BLE) broadcasting. This project allows multiple "Receiver" devices to execute actions in almost perfect sync, triggered by a central "Sender".

## Overview

This system minimize wireless latency by separating the command transmission from the execution time.

1. The Sender broadcasts a command saying: "Do X in 4000ms".

2. The Receiver gets the message, calculates the exact remaining time, and sets a hardware timer.

3. The Result: Even if devices receive the packet at slightly different times, they all trigger the action at the exact same target timestamp.

## Getting Started

1. Two ESP32. One build and flash this project. The other one build and flash the project on https://github.com/jun16ee/ble_adv_API_test
2. Use the controller from https://github.com/jun16ee/bt_controller
3. Run the demo.