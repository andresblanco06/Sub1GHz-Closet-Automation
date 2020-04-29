//*****************************************************************************
//  SENSOR CONTROLLER STUDIO EXAMPLE: I2C TEMPERATURE AND HUMIDITY SENSOR FOR
//  LAUNCHPAD SENSORTAG KIT (LPSTK)
//  Operating system: TI-RTOS
//
//  Demonstrates use of the bit-banged I2C master interface by sampling the
//  HDC2080 temperature and humidity sensor on the CC1352R LaunchPad SensorTag
//  kit (LPSTK). The example also uses Timer 2 to pulse-width modulate (PWM)
//  the LPSTK's RGB LED.
//
//  The Sensor Controller's "I2C Temp and Humidity Sensor" task wakes up the
//  application on several events. The application then uses UART or the Sensor
//  Controller's "RGB LED Blinker" task to indicate the results.
//  - Error handling: I2C missing acknowledgment or SCL stretch timeout
//      - Continuous red LED blinks
//      - Reset the application to end this error condition
//  - Error handling: HDC2080 interrupt timeout
//      - Continuous blue LED blinks
//      - Reset the application to end this error condition
//  - Temperature change by more than a configurable amount
//      - A blue LED blink indicates temperature decrease
//      - A red LED blink indicates temperature increase
//  - Humidity change by more than a configurable amount
//      - A yellow LED blink indicates humidity decrease
//      - A green LED blink indicates humidity increase
//  - Temperature and humidity log buffers are full (every 10 seconds)
//      - All temperature and humidity values in the log buffer are printed
//        over UART (57600 baud, 8-N-1)
//
//  Use a terminal window to connect to the LaunchPad's XDS110 Application/User
//  USB serial port.
//
//
//  BOARD SETUP (requires a LaunchPad, for example LAUNCHXL-CC1352R1):
//  - Remove all 11 jumpers on the pin row between the XDS110 and the device on
//    the LaunchPad.
//  - Use the supplied cables to connect JTAG and UART from the LPSTK board to
//    the LaunchPad.
//
//
//  Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
//
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//    Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//****************************************************************************/
#include "ex_include_tirtos.h"
#include "scif.h"
#include "stdio.h"
#include "string.h"


#define BV(n)               (1 << (n))


// Display error message if the SCIF driver has been generated with incorrect operating system setting
#if !(defined(SCIF_OSAL_TIRTOS_H) || defined(SCIF_OSAL_TIDPL_H))
    #error "SCIF driver has incorrect operating system configuration for this example. Please change to 'TI-RTOS' or 'TI Driver Porting Layer' in the Sensor Controller Studio project panel and re-generate the driver."
#endif

// Display error message if the SCIF driver has been generated with incorrect target chip package
#ifndef SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ
    #error "SCIF driver has incorrect target chip package configuration for this example. Please change to 'QFN48 7x7 RGZ' in the Sensor Controller Studio project panel and re-generate the driver."
#endif


// Task data
Task_Struct myTask;
Char myTaskStack[1024];


// UART driver objects
UART_Params uParams;

// Buffer for UART printing
char pLine[64];


// Semaphore used to wait for Sensor Controller task ALERT event
static Semaphore_Struct semScTaskAlert;




void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback




void scTaskAlertCallback(void) {

    // Wake up the OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));

} // scTaskAlertCallback




void taskFxn(UArg a0, UArg a1) {

    // Initialize UART parameters
    UART_Params_init(&uParams);
    uParams.baudRate      = 57600;
    uParams.writeDataMode = UART_DATA_TEXT;
    uParams.dataLength    = UART_LEN_8;
    uParams.stopBits      = UART_STOP_ONE;

    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    // Start the Sensor Controller's I2C Temperature and Humidity Sensor task (not to be confused with
    // OS tasks)
    scifTaskData.i2cTempAndHumiditySensor.cfg.tempChangeThr = 10;
    scifTaskData.i2cTempAndHumiditySensor.cfg.humChangeThr = 10;
    scifStartTasksNbl(BV(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID) | BV(SCIF_RGB_LED_BLINKER_TASK_ID));
    scifWaitOnNbl(1000000);

    // Main loop
    int16_t prevTemp = 0x7FFF;
    uint16_t prevHum = 0xFFFF;
    while (1) {

        // Wait for an ALERT callback
        Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

        // Clear the ALERT interrupt source
        scifClearAlertIntSource();

        // If I2C error occurred, indicate this by continuous red LED blinking ...
        uint16_t bvReport = scifTaskData.i2cTempAndHumiditySensor.output.bvReport;
        while (bvReport & SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_I2C_ERROR) {
            scifTaskData.rgbLedBlinker.input.pRedDutyCycle[0] = 64;
            scifSwTriggerExecutionCodeNbl(BV(SCIF_RGB_LED_BLINKER_TASK_ID));
            scifWaitOnNbl(1000000);
        }

        // If HDC2080 interrupt timeout occurred, indicate this by continuous blue LED blinking ...
        while (bvReport & SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_INT_TIMEOUT) {
            scifTaskData.rgbLedBlinker.input.pBlueDutyCycle[0] = 64;
            scifSwTriggerExecutionCodeNbl(BV(SCIF_RGB_LED_BLINKER_TASK_ID));
            scifWaitOnNbl(1000000);
        }

        // If there is a temperature or humidity change ...
        if (bvReport & (SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_TEMP_CHANGE |
                        SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_HUM_CHANGE)) {

            // Get the last results
            int16_t temp = scifTaskData.i2cTempAndHumiditySensor.output.temp;
            uint16_t hum = scifTaskData.i2cTempAndHumiditySensor.output.hum;

            // If the temperature has changed, generate red or blue blink
            if (bvReport & SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_TEMP_CHANGE) {
                if (temp > prevTemp) {
                    scifTaskData.rgbLedBlinker.input.pRedDutyCycle[0] = 64;
                } else {
                    scifTaskData.rgbLedBlinker.input.pBlueDutyCycle[0] = 32;
                }
                prevTemp = temp;
            }

            // If the humidity has changed, generate green or yellow blink
            if (bvReport & SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_HUM_CHANGE) {
                if (hum > prevHum) {
                    scifTaskData.rgbLedBlinker.input.pGreenDutyCycle[1] = 64;
                } else {
                    scifTaskData.rgbLedBlinker.input.pRedDutyCycle[1] = 64;
                    scifTaskData.rgbLedBlinker.input.pGreenDutyCycle[1] = 32;
                }
                prevHum = hum;
            }

            // Indicate temperature and/or humidity change by one or two LED blink(s)
            scifSwTriggerExecutionCodeNbl(BV(SCIF_RGB_LED_BLINKER_TASK_ID));
            scifWaitOnNbl(1000000);
        }

        // If the temperature and humidity log buffers are full, output it over UART
        if (bvReport & (SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_BV_REPORT_LOG_FULL)) {
            UART_Handle uHandle = UART_open(Board_UART, &uParams);
            for (int n = 0; n < SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_LOG_SIZE; n++) {
                int16_t temp = scifTaskData.i2cTempAndHumiditySensor.output.pTempLog[n];
                uint16_t hum = scifTaskData.i2cTempAndHumiditySensor.output.pHumLog[n];
                sprintf(pLine, "+%3.2f degC,  %3.2f%% RH\r\n", ((float) temp) / 64.0, ((float) hum) / 64.0);
                UART_write(uHandle, pLine, strlen(pLine));
            }
            UART_write(uHandle, "\r\n", 2);
            UART_close(uHandle);
        }

        // Acknowledge the alert event
        scifAckAlertEvents();
    }

} // taskFxn




int main(void) {
    Task_Params taskParams;

    // Initialize the board, with shutdown of external flash
    Board_init();

    // Initialize the UART driver
    UART_init();

    // Configure the OS task
    Task_Params_init(&taskParams);
    taskParams.stack = myTaskStack;
    taskParams.stackSize = sizeof(myTaskStack);
    taskParams.priority = 3;
    Task_construct(&myTask, taskFxn, &taskParams, NULL);

    // Create the semaphore used to wait for Sensor Controller ALERT events
    Semaphore_Params semParams;
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&semScTaskAlert, 0, &semParams);

    // Start TI-RTOS
    BIOS_start();
    return 0;

} // main
