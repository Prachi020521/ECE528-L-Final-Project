/**
 * @file BLE_UART.h
 *
 * @brief Header file for the BLE_UART driver.
 *
 * This file contains the function definitions for the BLE_UART driver.
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * The following connections must be made:
 *  - BLE UART MOD  (Pin 1)     <-->  MSP432 LaunchPad Pin P1.6
 *  - BLE UART CTS  (Pin 2)     <-->  MSP432 LaunchPad GND
 *  - BLE UART TXO  (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (PM_UCA3RXD)
 *  - BLE UART RXI  (Pin 4)     <-->  MSP432 LaunchPad Pin P9.7 (PM_UCA3TXD)
 *  - BLE UART VIN  (Pin 5)     <-->  MSP432 LaunchPad VCC (3.3V)
 *  - BLE UART RTS  (Pin 6)     <-->  Not Connected
 *  - BLE UART GND  (Pin 7)     <-->  MSP432 LaunchPad GND
 *  - BLE UART DFU  (Pin 8)     <-->  Not Connected
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Prachi Patel and Sean Felker
 *
 */

#ifndef INC_BLE_UART_H_
#define INC_BLE_UART_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "msp.h"
#include "Clock.h"
#include "inc/EUSCI_A0_UART.h"

// Specifies the size of the array used to store the received BLE data
#define BLE_UART_BUFFER_SIZE 128        // Used for sending strings
#define BLE_CONTROLLER_READING_SIZE 19  // Used for quaternion controller

// -Character definitions from ASCII table-

// Carriage return character ("\r")
#define CR 0x0D

// Line feed character - new line ("\n")
#define LF 0x0A

// Back space character
#define BS 0x08

// Escape character
#define ESC 0x1B

// Space character
#define SP 0x20

// Delete character
#define DEL 0x7F

/*
 * @brief Initializes EUSCI the EUSCI_A3 module for UART communication with the Adafruit BLE module
 * Configures pins P9.6 (PM_UCA3RXD) and P9.7 (PM_UCA3TXD) as the receive and transmit pins respectively
 * Configures pin P1.6 as an output GPIO to control the BLE module's mode.
 * Configures the necessary registers for UART settings and baud rate
 *
 * @param none
 *
 * @return none
 */
void BLE_UART_Init();

/*
 *@brief Function to read a byte that was received by the EUSCI_A3 module
 *
 *@param none
 *@param
 *@return The byte received by the EUSCI_A3 module
 */
uint8_t BLE_UART_InChar();


/*
 * @brief Function to send one byte from the EUSCI_A3 module to the connected device
 *
 * @param data: Byte to send to the connected device
 *
 * @return none
 */
void BLE_UART_OutChar(uint8_t data);


/*
 * @brief Function to read an entire string that was received by the EUSCI_A3 module
 * Calls the BLE_UART_InChar function in a loop until the carriage return character is read, indicating the end of the string
 *
 * @param buffer_pointer: Pointer to the array that the string will be stored in
 * @param buffer_size: Size of the array that the string will be stored in
 *
 * @return Size of the string that was received
 */
int BLE_UART_InString(char *buffer_pointer, uint16_t buffer_size);


/*
 * @brief Function to transmit an entire string out of the EUSCI_A3 module
 * Calls the BLE_UART_OutChar function in a loop
 *
 * @param pt: String/character array being sent
 *
 * @return none
 */
void BLE_UART_OutString(char *pt);


/*
 * @brief Function to check the received UART string for a matching string
 *
 * @param BLE_UART_Data_Buffer: Pointer to the received UART string
 * @param data_string: String/character array to check the UART data for
 *
 * @return Returns 0x01 if the string was found, 0x00 if not
 */
uint8_t Check_BLE_UART_Data(char BLE_UART_Data_Buffer[], char *data_string);


/*
 * @brief Resets the conncted Adafruit BLE module by sending the string "ATZ" in command mode
 *
 * @param none
 *
 * @return none
 */
void BLE_UART_Reset();


/*
 * @brief Receives a (quaternion) data packet from the connected BLE module
 * Calls the BLE_UART_InChar function in a loop.
 *
 * @param buffer_pointer: Pointer to the array of bytes that the packet will be stored in
 * @param buffer_size: Number of bytes the function will try to receive
 *
 * @return Size of the packet that was received
 */
int BLE_UART_Read_Q_Packet(uint8_t *buffer_pointer, uint16_t buffer_size);


/*
 *@brief Prints the 4 quaternion data values that were received and stored in an array
 *
 *@param buffer_pointer: Array that stores the quaternion data packet
 */
void Print_Quaternion(uint8_t *buffer_pointer);


/*
 * @brief Checks the checksum bit of the received data packet to determine if there are errors in the data packet
 *
 * @param buffer: Pointer to the array the received data is stored at
 *
 * @ return Returns 1 if the checksum test passed, 0 if it did not
 */
uint8_t checkCRC(uint8_t *buffer);



#endif /* INC_BLE_UART_H_ */
