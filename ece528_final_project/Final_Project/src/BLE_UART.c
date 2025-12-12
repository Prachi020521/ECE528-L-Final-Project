/**
 * @file BLE_UART.c
 *
 * @brief Source code for the BLE_UART driver.
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
 * @author Prachi Patel, Sean Felker
 *
 */

#include "../inc/BLE_UART.h"

void BLE_UART_Init()
{

    // Configure pins P9.6 (PM_UCA3RXD) and P9.7 (PM_UCA3TXD) to use primary module function
    P9->SEL0 |= 0xC0;
    P9->SEL1 &= 0xC0;

    // Configure P1.6 as an output GPIO
    P1->SEL0 &= ~0x40;
    P1->SEL1 &= ~0x40;
    P1->DIR |= 0x40;

    // Hold the EUSCI_A3 Module in the reset state by setting bit 0 of the CTLW0 register
    EUSCI_A3->CTLW0 |= 0x01;

    // Clear all bits in the Modulation Control Word register
    EUSCI_A3->MCTLW &= ~0xFF;

    // Disable the parity by by clearing the UCPEN bit (bit 15) in the CTLW0 register
    EUSCI_A3->CTLW0 &= ~0x8000;

    // Select odd parity
    EUSCI_A3->CTLW0 &= ~0x4000;

    // Set bit order to LSB first
    EUSCI_A3->CTLW0 &= ~0x2000;

    // Select 8-bit character length
    EUSCI_A3->CTLW0 &= ~0x1000;

    // Select one stop bit
    EUSCI_A3->CTLW0 &= ~0x0800;
    // Enable UART mode
    EUSCI_A3->CTLW0 &= ~0x0600;

    // Disable synchronous mode
    EUSCI_A3->CTLW0 &= ~0x0100;

    // Configure the EUSCI_A3 to use SMCLK as the clock source (12 MHz)
    EUSCI_A3->CTLW0 |= 0x00C0;

    // Set the baud rate by writing to the UCBRx field (Bits 15 to 0)
    // 12 MHz / 9600 baud rate = 1250
    EUSCI_A3->BRW = 1250;

    // Disable the Transmit Complete and Start Bit Interrupts
    EUSCI_A3->IE &= ~0x0C;

    // Enable the Transmit Interrupt and Receive Interrupt
    EUSCI_A3->IE |= 0x03;

    // Release the EUSCI_A3 module from the reset state
    EUSCI_A3->CTLW0 &= ~0x01;
}

uint8_t BLE_UART_InChar()
{
    // Check if the Receive Interrupt FLag (UCRXIFG, Bit 0) in the
    // IFG register, wait until the flag is set
    // When the flag is set, UCAxRXBUF has received a complete character

    while((EUSCI_A3->IFG & 0x01) == 0){

    }

    return EUSCI_A3->RXBUF;
}

void BLE_UART_OutChar(uint8_t data)
{
    // Wait until the Transmit Interrupt Flag (UCTXIFG, Bit 1)
    // in the IFG register is set
    // If the UCTXIFG is set, then the transmit buffer (UCAxTXBUF) is empty
    while((EUSCI_A3->IFG & 0x02) == 0){

    }

    // Write the data that will be sent to the transmit buffer, clearing the UCTXIFG flag
    EUSCI_A3->TXBUF = data;
}




int BLE_UART_InString(char *buffer_pointer, uint16_t buffer_size)
{
    int length = 0;
    int string_size = 0;

    // Read the last received data from the UART receive buffer
    char character = BLE_UART_InChar();

    // Check if the received character is a carriage return
    while(character != LF){

        // Removes character from the buffer it is the backspace character
        if (character == BS){
            if(length){
                buffer_pointer--;
                length--;
                BLE_UART_OutChar(BS);
            }
        }

        // More characters to be read, store them in the buffer
        else if(length < buffer_size){
            *buffer_pointer = character;
            buffer_pointer++;
            length++;
            string_size++;
        }
        // Read the next character

        character = BLE_UART_InChar();
    }
    *buffer_pointer = 0;

    return string_size;
}


void BLE_UART_OutString(char *pt)
{

    // Transmit characters of the string until the null pointer is reached indicating the end of the string
    while(*pt){
        BLE_UART_OutChar(*pt);
        pt++;
    }
}

uint8_t Check_BLE_UART_Data(char BLE_UART_Data_Buffer[], char *data_string)
{
    // Check if a match for the string in data_string is found in the BLE_UART_Data_Buffer[] string
    if (strstr(BLE_UART_Data_Buffer, data_string) != NULL){
        return 0x01;
    }
    else{
        return 0x00;
    }
}

void BLE_UART_Reset()
{

    // Switch to CMD mode by setting the MOD pin (P1.6) to 1
    P1->OUT |= 0x40;
    Clock_Delay1ms(1000);

    // Send the system reset command by sending the "ATZ" string to the
    // BLE UART module
    BLE_UART_OutString("ATZ\r\n");
    Clock_Delay1ms(3000);

    // Switch back to DATA mode by clearing the MOD pin to 0
    P1->OUT &= ~0x40;
}

int BLE_UART_Read_Q_Packet(uint8_t *buffer_pointer, uint16_t buffer_size)
{
    int length = 0;
    int packet_size = 0;

    // Read the last received data from the UART receive buffer
    for(int i = 0; i < buffer_size; i++){
        uint8_t byte = BLE_UART_InChar();

        // More data to be stored in the buffer
        if(length < buffer_size){
            *buffer_pointer = byte;
            buffer_pointer++;
            length++;
            packet_size++;
        }
    }
    return packet_size;
}

void Print_Quaternion(uint8_t *buffer_pointer)
{
    // Check for the Quaternion packet
    if( (char)buffer_pointer[1] == '!'){
        if( (char)buffer_pointer[2] == 'Q'){

            // Copy each quaternion value into its own variable
            float qx;
            memcpy(&qx, buffer_pointer + 3, 4);
            printf("qx: %f \r\n", qx);

            float qy;
            memcpy(&qy, buffer_pointer + 7, 4);
            printf("qy: %f \r\n", qy);

            float qz;
            memcpy(&qz, buffer_pointer + 11, 4);
            printf("qz: %f \r\n", qz);

            float qw;
            memcpy(&qw, buffer_pointer + 15, 4);
            printf("qw: %f \r\n", qw);
        }
        else{
            printf("Quaternion not found\r\n");
        }
    }
    else{
        printf("Quaternion not found\r\n");
    }
}

uint8_t checkCRC(uint8_t *buffer){
    uint8_t len = sizeof(buffer);
    uint8_t crc = buffer[len - 1];
    uint8_t sum = 0;

    // Sums all data in the packet
    for(int i = 0; i < (len - 1); i++){
        sum = sum + buffer[i];
    }
    printf("CRC: ");
    if ((crc & ~sum) == 0){
        printf("PASS\r\n");
        return 1;
    }
    else{
        printf("FAIL\r\n");
        return 0;
    }

}

