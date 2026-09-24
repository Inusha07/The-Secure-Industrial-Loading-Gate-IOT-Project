# Secure Industrial Loading Gate

## Module
COMP50069 – Hardware, Microcontrollers and Sensors

## Scenario
Scenario 1 – The Secure Industrial Loading Gate

## Description
This project implements an ESP32-based secure industrial loading gate.

The system uses:
- ESP32
- PIR sensor
- HC-SR04 ultrasonic sensor
- Potentiometer
- Servo motor
- Buzzer
- I2C OLED display

## Operating Modes

### Autonomous Mode
The PIR sensor detects motion and opens the gate.
The potentiometer controls the gate hold-open delay.
The gate closes automatically when the path is clear.

### Manual Mode
The operator can control the gate through the Serial Monitor.

Commands:

- A – Autonomous Mode
- M – Manual Mode
- O – Open Gate
- C – Close Gate
- H – Hold Gate
- S – Safety Test
- R – Reset

### Safety Mode
If the ultrasonic sensor detects an obstruction below 20 cm while
the gate is closing, the gate stops and enters Safety mode.

The buzzer provides a warning and the gate reopens.
The system returns to Autonomous mode after the path is clear.

## Hardware Connections

| Component | ESP32 Pin |
|---|---|
| PIR OUT | GPIO 27 |
| HC-SR04 TRIG | GPIO 5 |
| HC-SR04 ECHO | GPIO 18 |
| Potentiometer | GPIO 34 |
| Servo Signal | GPIO 13 |
| Buzzer | GPIO 25 |
| OLED SDA | GPIO 21 |
| OLED SCL | GPIO 22 |

## Software

Developed using:
- Arduino IDE
- ESP32
- C/C++

## Communication

Serial Monitor:

115200 baud

## Main Features

- Digital I/O
- UART communication
- ADC input
- PWM servo control
- I2C OLED
- Non-blocking timing
- Hardware timer
- Autonomous mode
- Manual mode
- Safety mode
