# Power Rail Layout

This document describes the power distribution for the project.

---

## Overview

The system uses two main power rails:

- **RAIL 1 – 12V Battery Pack**: Drives high-current loads (motors, pump).
- **RAIL 2 – 5V Logic**: Powers the ESP32 and low-power sensors/actuators.

> **Important:** All grounds **must** be connected together to ensure proper reference for signals between modules.

---

## RAIL 1: 12V Battery Pack (Motors + Pump)

**Source:** 12V battery pack

**Connections:**

- `+12V`  
  - → `L298N VCC` (12V motor supply input)  
  - → `Pump Positive` (via MOSFET or relay module)

- `GND`  
  - → `L298N GND`  
  - → `Pump GND`  
  - → **Common Ground** (must be tied to RAIL 2 GND)

**Notes:**

- Ensure wiring and connectors are rated for the motor and pump current.
- Place a suitable fuse on the +12V line near the battery pack for protection.
- Consider adding a bulk capacitor (e.g., 470µF–1000µF, 25V) across 12V and GND near motor driver and pump to help with voltage dips.

---

## RAIL 2: 5V Logic (ESP32 + Sensors + Servo)

**Source options:**

- Dedicated **5V regulator** from the 12V battery (e.g., buck converter), or  
- USB 5V (for ESP32) plus external 5V supply for servos/sensors.

**Connections:**

- `+5V`  
  - → `ESP32 VIN` (or via USB 5V, depending on board)  
  - → `Servo Power` (recommended separate 5V 2A regulator for servos)  
  - → `Sensors` (as required, respecting each sensor’s voltage spec)

- `GND`  
  - → `ESP32 GND`  
  - → `Servo GND`  
  - → `Sensor GND`  
  - → **Connect to RAIL 1 GND** (common ground)

**Notes:**

- Servos can produce large current spikes; use an external 5V 2A (or higher) supply and decoupling capacitors (e.g., 470µF) close to the servo power rail.
- Do **not** power servos directly from the ESP32 5V pin if current draw is high.
- Confirm the ESP32 board’s VIN requirements (commonly 5V) and do not exceed its ratings.

---

## Common Ground Requirement

All GND lines from:

- 12V battery pack (RAIL 1)  
- 5V logic supply (RAIL 2)  
- L298N motor driver  
- Pump driver (MOSFET/relay)  
- ESP32  
- Servos and sensors  

must be connected together at a common ground reference.

This ensures:

- Reliable signal levels between ESP32 and motor/pump drivers.
- Correct operation of PWM and control signals.
- Prevention of floating reference issues that can lead to erratic behavior.

---

## Example Block Diagram (Textual)

- **12V Battery (RAIL 1)**  
  - +12V → L298N VCC  
  - +12V → Pump (via MOSFET/Relay)  
  - GND → L298N GND  
  - GND → Pump GND  
  - GND → Common Ground Node

- **5V Logic (RAIL 2)**  
  - 5V Regulator / USB → ESP32 VIN  
  - 5V Regulator (2A+) → Servo +5V  
  - 5V → Sensors  
  - GND (from 5V source) → ESP32 GND  
  - GND → Servo GND  
  - GND → Sensor GND  
  - GND → Common Ground Node (tied to RAIL 1 GND)

---
