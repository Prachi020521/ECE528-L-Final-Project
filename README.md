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



  
