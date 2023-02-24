/* CS350 Project Submission By Paul */

/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * PROBLEM
 *
 *  Code must:
 *  - Check whether buttons have been pressed every 200ms
 *  - Check the temperature every 500ms
 *  - Update the LED and report to the server every second (via the UART)
 *  - Increase/decrease set-point temperature when buttons are pressed
 *  - LED must turn on when the room temperature is less than the set-point temperature
 *  - LED must turn off when the room temperature is greater than or equal to the set-point temperature
 *  - Output to the server must be formatted as <AA,BB,S,CCCC> where,
 *          AA = ASCII decimal value of room temperature (00 - 99) degrees Celsius
 *          BB = ASCII decimal value of set-point temperature (00-99) degrees Celsius
 *          S = ‘0’ if heat is off, ‘1’ if heat is on
 *          CCCC = decimal count of seconds since board has been reset
 * */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h> // snprintf()
#include <stdlib.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// Macros
#define DISPLAY(x) UART_write(uart, &output, x);
#define TRUE 1
#define FALSE 0
#define BEGIN 0
#define NUMBER_OF_TASKS 3
#define GLOBAL_PERIOD 100 // ms

// Global variables
int16_t current_temp = BEGIN;
int16_t set_temp = 25;
int16_t left_button = FALSE;
int16_t right_button = FALSE;
int16_t heat_on = FALSE;

unsigned long seconds_passed = BEGIN;

// Global flag
volatile unsigned char ready_tasks = FALSE;

// Global period used by initTimer()
int global_period = GLOBAL_PERIOD;

// UART Global Variables
char output[64];
int bytesToSend;

// I2C Global Variables
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
}

sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
UART_Handle uart;
I2C_Handle i2c;
Timer_Handle timer0;

// Forward declaration
enum CB_States { CB_SMStart, CB_s0, CB_s1, CB_s2} CB_State;
void CheckButtons();

enum CT_States { CT_SMStart, CT_s1, CT_s2} CT_State;
void CheckTemp();

enum OTS_States { OTS_SMStart, OTS_s1} OTS_State;
void OutputToServer();

/* SOLUTION: Required functionality divided into 3 tasks, each with their own period, tick function, and elapsed time variable */

// Struct to collect all items related to a task
struct task_entry
{
    void(*f)();       // Function to call to perform the task
    int elapsed_time; // Amount of time since last trigger
    int period;       // Period of the task in ms
};

// Task list
struct task_entry tasks[NUMBER_OF_TASKS] =
{
    {&CheckButtons, BEGIN, 200},
    {&CheckTemp, BEGIN, 500},
    {&OutputToServer, BEGIN, 1000}
};

/* SOLUTION: Created a timer that invokes a callback function every 100 ms to facilitate measuring elapsed time*/
/*
 *  ======== timerCallback ========
 *  Function that is invoked every time the timer expires.
 *  Setting ready_tasks to true raises the flag.
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    ready_tasks = TRUE;
}

/*
 *  ======== initUART ========
 *  Function to initialize timer.
 */
void initTimer(void)
{
    Timer_Params params;

    // Init the driver
    Timer_init();

    // Configure the driver
    Timer_Params_init(&params);

    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

/*
 *  ======== initUART ========
 *  Function to initialize UART peripheral
 *
 *  Make sure to call initUART() before calling this function.
 */
void initUART(void)
{
    UART_Params uartParams;

    // Init the driver
    UART_init();

    // Configure the driver
    UART_Params_init(&uartParams);

    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.baudRate = 115200;

    // Open the driver
    uart = UART_open(CONFIG_UART_0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

/*
 *  ======== initI2C ========
 *  Function to initialize I2C peripheral
 *
 *  Make sure to call initUART() before calling this function.
 */
void initI2C(void)
{
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))
    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;

    for (i=0; i<3; ++i)
    {
        i2cTransaction.slaveAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }

        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)
    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.slaveAddress))
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

/*
 *  ======== readTemp ========
 *  Function returns temperature from sensor in Celsius
 */
int16_t readTemp(void)
{
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor data sheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }

    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }

    return temperature;
}

/* SOLUTION: Capture the event when the left button is pressed */

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    left_button = TRUE;
}

/* SOLUTION: Capture the event when the right button is pressed */

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    right_button = TRUE;
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    initUART(); // Must be called before initI2C()
    initI2C();
    initTimer();

    // Set initial states for SMs
    CB_State = CB_SMStart;
    CT_State = CT_SMStart;

    /* SOLUTION: Implement task scheduler functionality to handle multiple tasks with varying periods. A
     * continuous loop iterates through a list of tasks, checking if their period has been reached. A timer is
     * used to measure the elapsed time and triggers a callback function every 100 ms. When the callback function is
     * triggered, a global flag is raised and all tasks are checked to see if their period has transpired.
     * If a task's period has not been reached, then its elapsed time is incremented by 100 ms. Otherwise, its
     * tick function is called and executed, and its elapsed time is reset to 0. The global flag is lowered so that
     * another 100 ms can go by before the task scheduler iterates through the list of tasks again.
     * */

    // Infinite loop for main processing
    while(TRUE)
    {
        // Wait for global flag to be raised
        while(!ready_tasks){}

            // Iterate through each task
            int x = BEGIN;
            for (x = BEGIN; x < NUMBER_OF_TASKS; x++)
            {
                // Check if interval has expired
                if (tasks[x].elapsed_time >= tasks[x].period)
                {
                    // Call function for each task that is triggered
                    tasks[x].f();

                    // Reset elapsed time
                    tasks[x].elapsed_time = BEGIN;
                }

                // Increment elapsed time for task by global period
                tasks[x].elapsed_time += global_period;
            }

            // Increment elapsed time since last restart by global period
            seconds_passed += global_period;

            // Reset global flag
            ready_tasks = FALSE;
    }
}

/* SOLUTION: Check if the buttons have been pressed every 200 ms and appropriately update the value of the
 * set-point temperature variable set_temp */

/*
 *  ======== CheckButtons ========
 *  Function that checks if left or right button has been pressed.
 *
 *  Pressing the left button, CONFIG_GPIO_BUTTON_0, decreases the temperature set-point by 1 degree.
 *  Pressing the right button, CONFIG_GPIO_BUTTON_1, increases the temperature set-point by 1 degree.
 */
void CheckButtons()
{
    // For Testing Purposes
    //printf("Calling CheckButtons()\n\r");

    switch(CB_State) {      // Transitions
        case CB_SMStart:
            CB_State = CB_s0;
            break;
        case CB_s0:
            if (left_button && !right_button)
            {
                CB_State = CB_s1;
            }
            else if (right_button && !left_button)
            {
                CB_State = CB_s2;
            }
        case CB_s1:
            if (!right_button && !left_button)
            {
                CB_State = CB_s0;
            }
            break;
        case CB_s2:
            if (!right_button && !left_button)
            {
                CB_State = CB_s0;
            }
            break;
        default:
            CB_State = CB_SMStart;
    }

    switch(CB_State) {    // State actions
        case CB_s1:
            set_temp -= 1;
            left_button = FALSE;
            break;
        case CB_s2:
            set_temp += 1;
            right_button = FALSE;
            break;
        default:
            left_button = FALSE;
            right_button = FALSE;
            break;
    }
}

/* SOLUTION: Check the temperature every 500 ms and appropriately turn on and off the LED,
 * CONFIG_GPIO_LED_0. Also, set the value of the heater variable, heat_on, such that 1 means that
 * the LED is on (set_temp > current_temp) and 0 means that the LED is off (set_temp <= current_temp) */

/*
 *  ======== CheckTemp ========
 *  Function that checks the room temperature and turns on/off LED (heater).
 *
 *  The LED (heater) turns on when the set-point temperature (set_temp) is greater than the
 *  room temperature (current_temp) and turns off when it is less than or equal to room temperature.
 */
void CheckTemp()
{
    // For Testing Purposes
    //printf("Calling CheckTemp()\n\r");

    current_temp = readTemp(); // Get current temperature in Celsius

    switch(CT_State) {      // Transitions
         case CT_SMStart:
            CT_State = CT_s1;
            break;
         case CT_s1:
            if (set_temp <= current_temp)
            {
                CT_State = CT_s2;
            }
            break;
         case CT_s2:
             if (set_temp > current_temp)
             {
                CT_State = CT_s1;
             }
            break;
         default:
             CT_State = CT_SMStart;
       }

      switch(CT_State) {    // State actions
         case CT_s1:
             // Turn on heater - red LED
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
             heat_on = TRUE;
            break;
         case CT_s2:
             // Turn off heater - red LED
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
             heat_on = FALSE;
            break;
         default:
            break;
      }
}

/* SOLUTION: Use the UART peripheral to output the values of the current temperature, set-point
 * temperature, heater/LED, and seconds passed since the board has been reset */

/*
 *  ======== OutputToServer ========
 *  Function that outputs to the server (via UART).
 *  Formatted as <AA,BB,S,CCCC> where:
 *      AA = current_temp = ASCII decimal value of room temperature (00 - 99) degrees Celsius
 *      BB = set_temp = ASCII decimal value of set-point temperature (00-99) degrees Celsius
 *      S = heat_on = ‘0’ if heat is off, ‘1’ if heat is on
 *      CCCC = seconds_passed / 1000 = decimal count of seconds since board has been reset
 */
void OutputToServer()
{
    // For Testing Purposes
    //printf("Calling OutputToServer()\n\r");

    switch(OTS_State) {      // Transitions
        case OTS_SMStart:
            OTS_State = OTS_s1;
            break;
        case OTS_s1:
            break;
        default:
            OTS_State = OTS_SMStart;
    }

    switch(OTS_State) {    // State actions
        case OTS_s1:
            DISPLAY(snprintf(output, 64, "<%02d, %02d, %d, %04d> \n\r", current_temp, set_temp, heat_on, seconds_passed / 1000))
            OTS_State = OTS_SMStart;
            break;
        default:
            break;
    }
}
