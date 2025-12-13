# Tilt/Gesture Controlled Robot

## Introduction
This is the final project assignment for ECE 528/ECE528L Fall 2025 at California State University, Northridge

Designed By:
- Prachi Patel
- Sean Felker

Professor:
- Aaron Nanas

## Objective
Advancement in human machine interaction technologies allow human to communicate more effectively with robot and electrical devices. This project main goal is to design a sensor based hand gesture controlled robot that responses the user’s physical hand movement or tilting of a smartphone, which enables motions such as forward, backward, left, right, or diagonal without using traditional input devices such as remote or joysticks. The smartphone has built-in motion and orientation sensor that provide quaternion reading through BluefruitConnect app. This quaternion sensor readings transmit to Adafruit bluefruit LE UART module wirelessly and then the bluefruit LE UART module will transmit that received reading to the MSP432P401R microcontroller via UART serial communication. The MSP432P401R microcontroller drive the robot to desired direction by adjusting PWM signals accordingly the data that received. The additional feature included is the speed control mode which controls speed of the motor depending on how much the smartphone is tilted. 

## Block Diagram
![alt text](<Untitled Diagram.drawio.png>)

## Table of Components

|  |  |  |
| --- | --- | --- |
| **Description** | **Quantity** | **Manufacturer** |
| MSP432 LaunchPad | 1 | Texas Instruments |
| USB-A to Micro-USB Cable | 1 | N/A |
| TI-RSLK MAX Chassis | 1 | Pololu |
| Adafruit BLE UART Module | 1 | Adafruit |
| Jumper Wires | 6 | N/A |
| iOS Smartphone | 1 | Foxconn, Pegatron |
| 3.3 V DC Power Supply | 1 | Texas Instruments |

# Table of Pinouts


- Pins for Motor and RGB LED:

|  |  |  |
| --- | --- | --- |
| **Pin Label** | **Connection** | **Purpose** |
| P2.0, P2.1, P2.2 | Direct to MSP432 LaunchPad | RGB LED to indicate command status |
| P2.6 | Direct to MSP432 LaunchPad | Right motor PWM |
| P2.7 | Direct to MSP432 LaunchPad | Left motor PWM |
| P3.6 | Direct to MSP432 LaunchPad | Right motor sleep |
| P3.7 | Direct to MSP432 LaunchPad | Left motor sleep |
| P5.4 | Direct to MSP432 LaunchPad | Left motor direction |
| P5.5 | Direct to MSP432 LaunchPad | Right motor direction |
| P1.1, P1.4 | Direct to MSP432 LaunchPad | Pushbutton on chassis to switch mode |

- Adafruit BLE UART Module Pinout:

|  |  |  |
| --- | --- | --- |
| **Adafruit BLE UART Module** | **Connection** | **Purpose** |
| MOD | P1.6 of the MSP432 LaunchPad | To select between the two modes: Command and Data |
| CTS | GND | BLE module can send data back to the microcontroller |
| TX0 | P9.6(PM\_UCA3RXD) | UART transmit pin from BLE to the microcontroller |
| RX1 | P9.7(PM\_UCA3TXD) | UART receive pin into BLE module from the microcontroller |
| VIN | 3.3V | Power supply for BLE module |
| RTS | Not Connected(N/C) | When it is fine to send data to the BLE module, this pin will be low. |
| GND | Ground | Common ground for BLE module |
| DFU | Not Connected(N/C) | To force the BLE module to enter a special firmware update mode |