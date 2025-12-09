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
#include <stdlib.h>

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


//#define TEST_PROPORTIONAL_CONTROL
//#define TEST_HORIZONTAL_CONTROL

void Process_BLE_UART_DATA_Vertical(float qx, float qy, float qz, float qw)
{

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
        LED2_Output(RGB_LED_WHITE);
    }
    // Backward + Right
    else if( (-0.65 < qx) && (qx < -0.1) && (0.1 < qy) && (qy < 0.65) )
    {
        Motor_Backward(PWM_NOMINAL, PWM_SLOW);
        LED2_Output(RGB_LED_SKY_BLUE);
    }
    // Backward + Left
    else if( (-0.65 < qx) && (qx < -0.1) && (-0.65 < qy) && (qy < -0.1) )
    {
        Motor_Backward(PWM_SLOW, PWM_NOMINAL);
        LED2_Output(RGB_LED_SKY_BLUE);
    }
    // Forward
    else if( (0.1 < qx) &&  (qx < 0.65))
    {
        LED2_Output(RGB_LED_GREEN);
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
    // Backward
    else if ( (-0.65 < qx) && (qx < -0.1))
    {
        LED2_Output(RGB_LED_BLUE);
        Motor_Backward(PWM_NOMINAL, PWM_NOMINAL);
    }
    // Right
    else if ( (0.1 < qy) && (qy < 0.65))
    {
        LED2_Output(RGB_LED_PINK);
        Motor_Right(2300, 2300);
    }
    // Left
    else if( (-0.65 < qy) && (qy < -0.1))
    {
        LED2_Output(RGB_LED_YELLOW);
        Motor_Left(2300, 2300);
    }
    else{
        Motor_Stop();
        LED2_Output(RGB_LED_RED);
    }
}


void Process_BLE_UART_DATA_Horizontal(float qx, float qy, float qz, float qw)
{
    // Forward + Right
    if ( (0.1 < qx) &&  (qx < 0.65) && (0.1 < qy) && (qy < 0.65) )
    {
        Motor_Forward(PWM_NOMINAL, PWM_SLOW);
        LED2_Output(RGB_LED_WHITE);

    }
    // Forward + Left
    else if( (-0.65 < qx) && (qx < -0.1) && (0.1 < qy) && (qy < 0.65) )
    {
        Motor_Forward(PWM_SLOW, PWM_NOMINAL);
        LED2_Output(RGB_LED_WHITE);
    }
    // Backward + Right
    else if( (0.1 < qx) &&  (qx < 0.65) && (-0.65 < qy) && (qy < -0.1) )
    {
        Motor_Backward(PWM_NOMINAL, PWM_SLOW);
        LED2_Output(RGB_LED_SKY_BLUE);
    }
    // Backward + Left
    else if( (-0.65 < qx) && (qx < -0.1) && (-0.65 < qy) && (qy < -0.1) )
    {
        Motor_Backward(PWM_SLOW, PWM_NOMINAL);
        LED2_Output(RGB_LED_SKY_BLUE);
    }
    // Forward
    else if( (0.1 < qy) && (qy < 0.65))
    {
        LED2_Output(RGB_LED_GREEN);
        Motor_Forward(PWM_NOMINAL, PWM_NOMINAL);
    }
    // Backward
    else if ( (-0.65 < qy) && (qy < -0.1))
    {
        LED2_Output(RGB_LED_BLUE);
        Motor_Backward(PWM_NOMINAL, PWM_NOMINAL);
    }
    // Right
    else if ( (0.1 < qx) && (qx < 0.65))
    {
        LED2_Output(RGB_LED_PINK);
        Motor_Right(2300, 2300);
    }
    // Left
    else if( (-0.65 < qx) && (qx < -0.1))
    {
        LED2_Output(RGB_LED_YELLOW);
        Motor_Left(2300, 2300);
    }
    else{
        Motor_Stop();
        LED2_Output(RGB_LED_RED);
    }

}

// NOT TESTED: Calculate PWM duty cycles proportional to how much the phone is tilted
int Calculate_PWM(float qx, float qy, uint8_t mode)
{
    float PWM_Value_f;
    int PWM_Value;
    float PWM_Value_2;

    // Forward or backward
    if(mode == 0){
        PWM_Value_f = abs(qx * 10000) - 500;
    }

    // Left or right
    else if(mode == 1){
        PWM_Value_f = abs(qy * 10000) - 500;
    }

    // Diagonal high value: If turning right + forward/backward, the LEFT motor gets this value
    //                      If turning left + forward/backward, the RIGHT motor gets this value
    else if(mode == 2){
        PWM_Value_f = abs(qx * 10000);
        PWM_Value_2 = abs(qy * 10000);
        PWM_Value_f = PWM_Value_f + PWM_Value_2 - 1000;
    }

    // Diagonal low value: If turning right + forward/backward, the RIGHT motor gets this value
    //                     If turning left + forward/backward, the LEFT motor gets this value
    else if(mode == 3){
        PWM_Value_f = abs(qx * 10000);
        PWM_Value_2 = abs(qy * 10000);
        PWM_Value_f = PWM_Value_f + PWM_Value_2 - 1000;

        PWM_Value_f = PWM_Value_f - PWM_Value_f * (1/3);
    }

    PWM_Value = (int)PWM_Value_f;
    return PWM_Value;
}

void Process_BLE_UART_DATA_Proportional(float qx, float qy, float qz, float qw)
{
    uint16_t PWM_Right;
    uint16_t PWM_Left;
    // Forward + Right
    if ( (0.1 < qx) &&  (qx < 0.65) && (0.1 < qy) && (qy < 0.65) )
    {
        PWM_Right = Calculate_PWM(qx, qy, 3);
        PWM_Left = Calculate_PWM(qx, qy, 2);
        Motor_Forward(PWM_Left, PWM_Right);
        LED2_Output(RGB_LED_WHITE);

    }
    // Forward + Left
    else if( (0.1 < qx) &&  (qx < 0.65) && (-0.65 < qy) && (qy < -0.1) )
    {
        PWM_Right = Calculate_PWM(qx, qy, 2);
        PWM_Left = Calculate_PWM(qx, qy, 3);
        Motor_Forward(PWM_Left, PWM_Right);
        LED2_Output(0x08);
    }
    // Backward + Right
    else if( (-0.65 < qx) && (qx < -0.1) && (0.1 < qy) && (qy < 0.65) )
    {
        PWM_Right = Calculate_PWM(qx, qy, 3);
        PWM_Left = Calculate_PWM(qx, qy, 2);
        Motor_Backward(PWM_Left, PWM_Right);
        LED2_Output(0x09);
    }
    // Backward + Left
    else if( (-0.65 < qx) && (qx < -0.1) && (-0.65 < qy) && (qy < -0.1) )
    {
        PWM_Right = Calculate_PWM(qx, qy, 2);
        PWM_Left = Calculate_PWM(qx, qy, 3);
        Motor_Backward(PWM_Left, PWM_Right);
        LED2_Output(0x0A);
    }
    // Forward
    else if( (0.1 < qx) &&  (qx < 0.65))
    {
        LED2_Output(RGB_LED_GREEN);
        PWM_Right = Calculate_PWM(qx, qy, 0);
        PWM_Left = Calculate_PWM(qx, qy, 0);
        Motor_Forward(PWM_Left, PWM_Right);
    }
    // Backward
    else if ( (-0.65 < qx) && (qx < -0.1))
    {
        LED2_Output(RGB_LED_BLUE);
        PWM_Right = Calculate_PWM(qx, qy, 0);
        PWM_Left = Calculate_PWM(qx, qy, 0);
        Motor_Backward(PWM_Left, PWM_Right);
    }
    // Right
    else if ( (0.1 < qy) && (qy < 0.65))
    {
        LED2_Output(RGB_LED_PINK);
        PWM_Right = Calculate_PWM(qx, qy, 1);
        PWM_Left = Calculate_PWM(qx, qy, 1);
        Motor_Right(PWM_Left, PWM_Right);
    }
    // Left
    else if( (-0.65 < qy) && (qy < -0.1))
    {
        LED2_Output(RGB_LED_SKY_BLUE);
        PWM_Right = Calculate_PWM(qx, qy, 1);
        PWM_Left = Calculate_PWM(qx, qy, 1);
        Motor_Left(PWM_Left, PWM_Right);
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

    // Initialize the two on board buttons
    Buttons_Init();

    // Initialize the chassis LEDs
    Chassis_Board_LEDs_Init();

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

    // Quaternion data values
    float qx;
    float qy;
    float qz;
    float qw;

    int packet_size;            // Packet size (for debugging)
    uint8_t crc;                // Checksum (not used)
    uint8_t buttons_status;     // Two on board buttons' status
    uint8_t drive_mode = 0;     // Vertical phone control (0) or horizontal phone control (1)

    while(1)
    {

        packet_size = BLE_UART_Read_Q_Packet(BLE_UART_Buffer, BLE_CONTROLLER_READING_SIZE);
        printf("BLE UART Data: ");

        // Copy the original data without the first element
        // if the first byte is not '!'
        if(BLE_UART_Buffer[0] != 0x21){
            for(int i = 0; i < BLE_CONTROLLER_READING_SIZE-1; i++){
                BLE_UART_Buffer_Clean[i] = BLE_UART_Buffer[i+1];
            }
        }
        for (int i = 0; i < 2; i++){
            printf("%c, ", BLE_UART_Buffer_Clean[i]);
        }

        // Store the quaternion values each into a 4 byte float variable
        memcpy(&qx, BLE_UART_Buffer_Clean + 2, 4);
        printf("%f ", qx);


        memcpy(&qy, BLE_UART_Buffer_Clean + 6, 4);
        printf("%f ", qy);


        memcpy(&qz, BLE_UART_Buffer_Clean + 10, 4);
        printf("%f ", qz);


        memcpy(&qw, BLE_UART_Buffer_Clean + 14, 4);
        printf("%f ", qw);

        printf("\r\n");
        printf("Packet size = %d\r\n", packet_size);

        printf("BLE UART char data: ");
        for (int i = 0; i < BLE_CONTROLLER_READING_SIZE; i++)
        {
            printf("%c", BLE_UART_Buffer[i]);
        }
        printf("\r\n");

        //crc = checkCRC(BLE_UART_Buffer);
        Print_Quaternion(BLE_UART_Buffer);



#ifdef TEST_HORIZONTAL_CONTROL

        buttons_status = Get_Buttons_Status();
        if(buttons_status != 0x12){     // If any buttons are pressed, switch control modes
            if(drive_mode == 0){
                drive_mode = 1;               // Horizontal control mode
                P8->OUT |= 0x21;        // Turn on the yellow chassis lights
                P8->OUT &= ~0xC0;       // Turn off the red chassis lights
            }
            else{
                drive_mode = 0;               // Vertical control mode
                P8->OUT |= 0xC0;        // Turn on the red chassis lights
                P8->OUT &= ~0x21;       // Turn off the yellow chassis lights
            }
        }

        if(drive_mode == 1){
            Process_BLE_UART_DATA_Horizontal(qx, qy, qz, qw);
        }
        else{
            Process_BLE_UART_DATA(qx, qy, qz, qw);
        }

#elif defined(TEST_PROPORTIONAL_CONTROL)
        Process_BLE_UART_DATA_Proportional(qx, qy, qz, qw);

#else
        Process_BLE_UART_DATA_Vertical(qx, qy, qz, qw);
#endif

        Clock_Delay1ms(100);

    }
}
