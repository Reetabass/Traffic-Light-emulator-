# Smart Traffic Light Controller with Pedestrian Detection

## Overview

This project implements a fully embedded **Smart Traffic Light System** on an ATmega328P microcontroller. The system operates in multiple modes (Auto, Manual, Emergency), controls traffic lights using PWM brightness, detects pedestrians using a sonar distance sensor, and allows full system monitoring via USART serial communication. 

The system demonstrates integration of multiple AVR modules including:  
✅ ADC  
✅ External Interrupts  
✅ Pin Change Interrupts  
✅ Timers (Overflow & PWM)  
✅ USART with interrupt-driven RX  
✅ Sonar (Ultrasonic Distance Measurement)  
✅ PWM-based LED brightness control  
✅ Buzzer alert generation

All modules are programmed directly with low-level AVR C — no Arduino libraries used.

---

## Project Features

- **Multiple Operational Modes:**
  - Auto Mode: Cycles lights automatically with pedestrian detection.
  - Manual Mode: User-controlled light switching.
  - Emergency Mode: Locks system to green for emergency flow.
  
- **Sonar-based Pedestrian Detection:**
  - Ultrasonic sensor detects nearby pedestrians.
  - Activates red light when pedestrians are within configurable distance.

- **PWM-controlled Traffic Lights:**
  - Smooth brightness control of red, yellow, and green LEDs.
  - Adaptive brightness based on ambient light using ADC.

- **USART Debug Interface:**
  - Real-time monitoring of system status and sensor readings.
  - Allows user to control debug output levels.

- **Buzzer Alerts:**
  - Beeping rate varies depending on pedestrian distance.

---

## Hardware Connections

| Function         | AVR Pin |
|-------------------|---------|
| Green Light (PWM) | PB1 |
| Yellow Light (PWM)| PB2 |
| Red Light (PWM)   | PB3 |
| Mode Change Button| PD2 |
| Manual Light Button| PB0 |
| Emergency Mode Button | PD3 |
| Sonar Trigger     | PD5 |
| Sonar Echo        | PD4 |
| Buzzer Output     | PD6 |
| USART TX/RX       | PD0 / PD1 |
| Light Sensor (ADC)| PC2 |

---

## Software Modules Breakdown

- **Timers:**
  - Timer0: Overflow interrupt for periodic pedestrian detection checks.
  - Timer1: Fast PWM controlling Green/Yellow lights.
  - Timer2: Fast PWM controlling Red light.

- **ADC:**
  - Reads ambient light level for adaptive LED brightness.

- **External Interrupts (INT1):**
  - Emergency Mode toggle.

- **Pin Change Interrupts (PCINT0, PCINT18):**
  - Manual Mode light switching.
  - Mode switching.

- **USART:**
  - Serial debug output and runtime debug mode control via RX interrupt.

- **Sonar Module:**
  - Distance calculation for pedestrian detection with adaptive beep rates.

- **Buzzer:**
  - Audible alert when pedestrian detected, with variable beep intervals based on distance.

---
