/**
 * @file main.c
 *
 * @brief Main source code for the BLE_UART program.
 *
 * This file contains the main entry point for the BLE_UART program,
 * which is used to demonstrate the BLE_UART driver.
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author
 *
 */
#include <stdint.h>
#include <math.h>

#include "msp.h"
#include "inc/Clock.h"
#include "inc/CortexM.h"
#include "inc/GPIO.h"
#include "inc/EUSCI_A0_UART.h"
#include "inc/Motor.h"
#include "inc/BLE_UART.h"

// Constant PWM duty cycle for motors
#define PWM_NOMINAL 4500

void Process_BLE_UART_DATA(char BLE_UART_Buffer[]){

    // Check if the user has sent a valid command string
    // If so, perform the corresponding command
    if(Check_BLE_UART_Data(BLE_UART_Buffer, "LED green ")){
        LED2_Output(RGB_LED_GREEN);
    }
    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "RGB LED OFF")){
        LED2_Output(RGB_LED_OFF);
    }
    // Invalid command string
    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "MOVE FORWARD 30")){
        LED2_Output(RGB_LED_BLUE);
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
        Clock_Delay1ms(3000);
        Motor_Stop();
    }
    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "MOVE BACKWARD 30")){
        LED2_Output(RGB_LED_PINK);
        Motor_Backward(PWM_NOMINAL, PWM_NOMINAL);
        Clock_Delay1ms(3000);
        Motor_Stop();
    }
    else if(Check_BLE_UART_Data(BLE_UART_Buffer, "ROTATE 180")){
        LED2_Output(RGB_LED_WHITE);
        Motor_Left(2300, 2300);
        Clock_Delay1ms(2000);
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
        Clock_Delay1ms(2000);
        Motor_Stop();
    }
    else{
        printf("BLE UART command not found.\n");
    }
}


int main(void)
{
    // Disable interrupts during initialization
    DisableInterrupts();

    // Initialize the 48 Mhz clock
    Clock_Init48MHz();

    // Initialize the RGB LED
    LED2_Init();

    // Initialize the EUSCI_A0_UART module (USB port, printf back to host computer's terminal)
    EUSCI_A0_UART_Init_Printf();

    // Initialize the Adafruit Bluefruit LE UART Friend module
    BLE_UART_Init();

    // Initialize the DC Motors
    Motor_Init();

    // Initialize a buffer that will be used to receive command strings from the BLE module
    uint8_t BLE_UART_Buffer[BLE_CONTROLLER_READING_SIZE] = {0};
    uint8_t BLE_UART_Buffer_Clean[BLE_CONTROLLER_READING_SIZE-1] = {0};

    // Enable interrupts used by the modules
    EnableInterrupts();

    // Provide a short delay after initialization and reset the BLE module - will be in Data mode
    Clock_Delay1ms(1000);
    BLE_UART_Reset();

    // Send a message to the BLE module to check if the connection is stable
    BLE_UART_OutString("BLE UART Active.\r\n");
    Clock_Delay1ms(1000);

    while(1)
    {


        int packet_size = BLE_UART_Read_Q_Packet(BLE_UART_Buffer, BLE_CONTROLLER_READING_SIZE);
        printf("BLE UART Data: ");

        // Copy the original data without the first element
        if(BLE_UART_Buffer[0] != 0x21){
            for(int i = 0; i < BLE_CONTROLLER_READING_SIZE-1; i++){
                BLE_UART_Buffer_Clean[i] = BLE_UART_Buffer[i+1];
            }
        }
        for (int i = 0; i < 2; i++){
            printf("%c, ", BLE_UART_Buffer_Clean[i]);
        }
        float qx;
        memcpy(&qx, BLE_UART_Buffer_Clean + 2, 4);
        printf("%f ", qx);
        float qy;
        memcpy(&qy, BLE_UART_Buffer_Clean + 6, 4);
        printf("%f ", qy);
        float qz;
        memcpy(&qz, BLE_UART_Buffer_Clean + 10, 4);
        printf("%f ", qz);
        float qw;
        memcpy(&qw, BLE_UART_Buffer_Clean + 14, 4);
        printf("%f ", qw);




        printf("\r\n");
        printf("Packet size = %d\r\n", packet_size);
        uint8_t crc = checkCRC(BLE_UART_Buffer);
        Print_Quaternion(BLE_UART_Buffer);
        Clock_Delay1ms(100);
    }
}
