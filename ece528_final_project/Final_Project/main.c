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
#define PWM_SLOW    3000

void Process_BLE_UART_DATA(float qx, float qy, float qz, float qw){

    // Check if the user has sent a valid command string
    // If so, perform the corresponding command
    // Move forward from quaternion qx value

    // Forward + Right
    if ( (0.1 < qx) &&  (qx < 0.65) && (0.1 < qy) && (qy < 0.65) )
    {
        Motor_Forward(PWM_NOMINAL, PWM_SLOW);
        LED2_Output(RGB_LED_WHITE);

    }
    // Forward + Left
    else if( (0.1 < qx) &&  (qx < 0.65) && (-0.65 < qy) && (qy < -0.1) )
    {
        Motor_Forward(PWM_SLOW, PWM_NOMINAL);
        LED2_Output(0x08);
    }
    // Backward + Right
    else if( (-0.65 < qx) && (qx < -0.1) && (0.1 < qy) && (qy < 0.65) )
    {
        Motor_Backward(PWM_NOMINAL, PWM_SLOW);
        LED2_Output(0x09);
    }
    // Backward + Left
    else if( (-0.65 < qx) && (qx < -0.1) && (-0.65 < qy) && (qy < -0.1) )
    {
        Motor_Backward(PWM_SLOW, PWM_NOMINAL);
        LED2_Output(0x0A);
    }
    // Forward
    else if( (0.1 < qx) &&  (qx < 0.65))
    {
        LED2_Output(RGB_LED_GREEN);
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
    // Move backward from quaternion qx value
    else if ( (-0.65 < qx) && (qx < -0.1))
    {
        LED2_Output(RGB_LED_BLUE);
        Motor_Backward(PWM_NOMINAL, PWM_NOMINAL);
    }
    // Turn right at 90 degree and Move forward
    else if ( (0.1 < qy) && (qy < 0.65))
    {
        LED2_Output(RGB_LED_PINK);
        Motor_Right(2300, 2300);
    }
    // Turn left at 90 degree and move forward
    else if( (-0.65 < qy) && (qy < -0.1))
    {
        LED2_Output(RGB_LED_SKY_BLUE);
        Motor_Left(2300, 2300);
    }
    else{
        Motor_Stop();
        LED2_Output(RGB_LED_RED);
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

        Process_BLE_UART_DATA(qx, qy, qz, qw);

    }
}
