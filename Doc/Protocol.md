# STM32WB55 Auto-Balance Unicycle Control - Communication Protocol

This document describes the communication protocol used by STM32WB55RGV6 chip for controlling and monitoring the auto-balance unicycle system. For hardware setup, please refer to the [Hardware Setup](./Hardware.md) documentation.

## Overview

Main Board: STM32WB55RGV6

| Connector   | Connectee     | Protocol | Notes                                      |
|-------------|---------------|----------|--------------------------------------------|
| Main Board   | Hollow motor  | CAN      | Torque control and current Position           |
| Main Board   | Linear motor  | TBD | Position control  |
| Main Board   | IMU           | I²C  | Accelerometer and gyroscope readings      |
| Main Board   | PC | Bluetooth module     | Wireless communication (e.g., mobile app) |
| Main Board   | PC(SWV)            | SWD | Monitoring and debugging               |

## Protocol Details

### Hollow Motor (CAN)

CAN protocol follows the original protocol used by the manufacturer of the hollow motor (see [here](./中空系列电机控..明1%202.pdf), the English version is [here](./中空系列电机控..明1.pdf)). 

### Linear Motor (Undecided)

- TBD -

### IMU (I²C)

- **~~BNO055~~**: I2C port with address `0x28` (7-bit) or `0x50` (8-bit). Hand written driver.

### Bluetooth Module

Connection to the PC or mobile app via Bluetooth. Applying bluetooth template from CubeMX.

### SWV (Serial Wire)

Used for real-time monitoring and debugging. Configured via CubeMX to output relevant telemetry data. On port 0 in debug mode.


