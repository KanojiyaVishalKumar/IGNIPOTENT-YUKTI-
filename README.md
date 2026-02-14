# 🔥 IGNIPOTENT – Autonomous Fire Detection & Suppression Robot

![ESP32](https://img.shields.io/badge/MCU-ESP32-red)
![IoT](https://img.shields.io/badge/Category-IoT-blue)
![Robotics](https://img.shields.io/badge/Domain-Robotics-orange)
![Status](https://img.shields.io/badge/Status-Prototype-green)

---

## 🚨 Problem Statement

Traditional fire safety systems:

- Detect fire late  
- Use excessive water  
- Cannot move toward source  
- Cause major water damage  

We built **IGNIPOTENT** — a smart autonomous fire-fighting robot using ESP32 that:

- Detects fire using IR + Smoke + Temperature sensors  
- Navigates toward fire source  
- Extinguishes fire using targeted water spray  
- Minimizes water wastage  
- Works autonomously without human intervention  

---

## 🧠 System Architecture
## 🔁 Patrol Mode – Fire Response Algorithm
```text
PATROL MODE
   ↓
Scan Environment (IR + Smoke + Temp)
   ↓
Fire Detected?
   ↓
NO  → Continue Patrol
YES
   ↓
Confirm Using Multi-Sensor Logic
   ↓
Move Toward Fire
   ↓
Activate Pump (Water Spray)
   ↓
Recheck Sensors
   ↓
Fire Extinguished?
   ↓
YES → Return to Patrol
NO  → Repeat Suppression
```

---

## ⚙️ Hardware Components

| Component | Quantity |
|-----------|----------|
| ESP32 Dev Board | 1 |
| IR Flame Sensor | 1 |
| MQ-2 Smoke Sensor | 1 |
| Temperature Sensor (DHT11 / LM35) | 1 |
| L298N Motor Driver | 1 |
| DC Motors + Wheels | 2 |
| Water Pump (5–12V) | 1 |
| Relay Module | 1 |
| Water Tank + Pipe | 1 |
| Chassis | 1 |
| Li-ion Battery | 1 |

---

## 🔌 Pin Configuration

| Component | ESP32 Pin |
|-----------|-----------|
| IR Sensor | GPIO 34 |
| MQ-2 Sensor | GPIO 35 |
| Temperature Sensor | GPIO 4 |
| Motor Left IN1 | GPIO 14 |
| Motor Left IN2 | GPIO 27 |
| Motor Right IN1 | GPIO 26 |
| Motor Right IN2 | GPIO 25 |
| Pump Relay | GPIO 33 |

---

## 🔍 Fire Detection Logic

Fire is confirmed only when:

- IR detects flame  
- Smoke sensor crosses threshold  
- Temperature rises above safe limit  

This **multi-sensor fusion logic** significantly reduces false alarms.

---

## 🚀 Features

- Autonomous navigation  
- Multi-sensor fire validation  
- Targeted water suppression  
- Low water consumption  
- Expandable to swarm using ESP-NOW  
- Future-ready for AI-based detection (ESP32-CAM)  

---

## 📊 Comparison with Traditional Systems

| Feature | IGNIPOTENT | Traditional Sprinkler |
|----------|------------|-----------------------|
| Mobility | ✅ Yes | ❌ No |
| Targeted Spray | ✅ Yes | ❌ No |
| Water Efficiency | High | Low |
| False Alarm Rate | Very Low | Moderate |
| Smart Integration | Yes | No |

---

## 🔮 Future Enhancements

- ESP-NOW mesh swarm (multiple robots)  
- AI flame detection with ESP32-CAM  
- Mobile app monitoring dashboard  
- GPS-based fire mapping  
- Solar charging dock  
- Cloud analytics integration  

---

## 👨‍💻 Team

**Team IGNIPOTENT**  
Hackathon Project 2026  
