/**
 * @file main.c
 *
 * @brief Main source code for the Tilt/Gesture Controlled Robot
 *
 * ECE 528/L Final Project
 * Professor: Aaron Nanas
 *
 * It interfaces with the Adafruit Bluefruit LE UART Friend Bluetooth Low Energy (BLE) module, which uses the UART communication protocol.
 *  - Product Link: https://www.adafruit.com/product/2479
 *
 * For more information on sensor data packets sent by the Bluefruit Connect app, refer to https://learn.adafruit.com/bluefruit-le-connect/controller
 *
 * For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Prachi Patel and Sean Felker
 *
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

// Uncomment one test at a time. Running with both tests commented results in the robot being controlled in vertical phone mode.

/* The Proportional Control test allows the user to control the speed of the robot for driving forward and backward or left and right turns.
 * The speed is dependent on how much the phone is tilted.
 */
//#define TEST_PROPORTIONAL_CONTROL


/* The Horizontal Control test allows the user to control the robot holding the phone in a horizontal direction
 * Either on board push button can be used to switch between horizontal and vertical control modes in this test.
*/
//#define TEST_HORIZONTAL_CONTROL


/*
 * @brief Function to drive the robot assuming the user is holding the phone in vertical/portrait mode
 *
 * The robot will drive in one of 8 directions depending on the tilt of the phone
 * The RGB LED color is also determined depending on the direction of the robot
 *
 * @param qx: Quaternion value to represent phone orientation in the x axis
 * @param qy: Quaternion value to represent phone orientation in the y axis
 * @param qz: Quaternion value to represent phone orientation in the z axis (NOT USED)
 * @param qw: Quaternion value to represent the magnitude of the phone's rotation from its original state (NOT USED)
 *
 * @ return none
 */
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


/*
 * @brief Function to drive the robot assuming the user is holding the phone in horizontal/landscape mode
 *
 * The robot will drive in one of 8 directions depending on the tilt of the phone
 * The RGB LED color is also turned on depending on the direction of the robot
 *
 * @param qx: Quaternion value to represent phone orientation in the x axis
 * @param qy: Quaternion value to represent phone orientation in the y axis
 * @param qz: Quaternion value to represent phone orientation in the z axis (NOT USED)
 * @param qw: Quaternion value to represent the magnitude of the phone's rotation from its original state (NOT USED)
 *
 * @ return none
 */
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

/*
 * @brief Function to calculate the PWM values to drive the robot motors for controlling the robot speed based on
 *        how much the phone is tilted
 *
 *  For driving the robot forward/backward and left/right, the same PWM value can be used for both motors (mode = 0 and mode = 1)
 *  For driving the robot in diagonal directions, the PWM value of one motor must be greater/smaller than the other (mode = 2 and mode = 3)
 *
 * @param qx: Quaternion value to represent phone orientation in the x axis
 * @param qy: Quaternion value to represent phone orientation in the y axis
 *
 * @return Calculated PWM value to drive to the motor
 */
int Calculate_PWM(float qx, float qy, uint8_t mode)
{
    float PWM_Value_f;
    int PWM_Value;
    float PWM_Value_2;

    // Forward or backward
    if(mode == 0){
        PWM_Value_f = abs(qx * 10000) - 500;    // Scale the qx value up to a PWM value
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


/*
 * @brief Process the quaternion data and drive the motors using the speed control mode.
 * Assumes the user is holding the phone in vertical/portrait mode
 * Uses the Calculate_PWM function to drive the motors with a speed based on how much the phone is tilted
 *
 *
 */
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


/*
 * @brief main function and loop for the tilt/gesture controlled robot
 *
 * Initializes the clock, LEDS, buttons, EUSCI modules, motors (TimerA0 + PWM), and necessary variables
 * The loop receives a packet from the BLE module into an array and processes the quaternion data into individual floating point variables.
 * If the robot is connected to the host computer, these values are printed.
 * Then, a function is called to drive the motors based on the quaternion qx and qy data.
 */

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
        // Receive and store the quaternion data packet
        packet_size = BLE_UART_Read_Q_Packet(BLE_UART_Buffer, BLE_CONTROLLER_READING_SIZE);
        printf("BLE UART Data: ");

        // Copy the original data without the first element
        // if the first byte is not '!'. Occurs in every packet from our testing
        if(BLE_UART_Buffer[0] != 0x21){
            for(int i = 0; i < BLE_CONTROLLER_READING_SIZE-1; i++){
                BLE_UART_Buffer_Clean[i] = BLE_UART_Buffer[i+1];
            }
        }

        // Print the first two characters of the data packet (!Q)
        for (int i = 0; i < 2; i++){
            printf("%c, ", BLE_UART_Buffer_Clean[i]);
        }

        // Copy each of the quaternion values from an array to a 4 byte float variable & print values
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

        // Prints original packet as characters
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
            Process_BLE_UART_DATA_Horizontal(qx, qy, qz, qw); // Horizontal control mode
        }
        else{
            Process_BLE_UART_DATA_Vertical(qx, qy, qz, qw); // Vertical control mode
        }

#elif defined(TEST_PROPORTIONAL_CONTROL)
        Process_BLE_UART_DATA_Proportional(qx, qy, qz, qw); // Speed control mode

#else
        Process_BLE_UART_DATA_Vertical(qx, qy, qz, qw); // Vertical control mode
#endif

        Clock_Delay1ms(100);    // Delay for the BLE module

    }
}
