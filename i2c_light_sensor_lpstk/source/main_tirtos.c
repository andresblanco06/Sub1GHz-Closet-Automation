//*****************************************************************************
//  SENSOR CONTROLLER STUDIO EXAMPLE: I2C LIGHT SENSOR FOR LAUNCHPAD SENSORTAG
//  KIT (LPSTK)
//  Operating system: TI-RTOS
//
//  Demonstrates use of the bit-banged I2C master interface by sampling the
//  OPT3001 light sensor on the CC1352R LaunchPad SensorTag Kit (LPSTK).
//
//  The application is woken if the light sensor output value changes by more
//  than a configurable amount:
//  - If decreasing, the application blinks the red LED on the LPSTK.
//  - If increasing, the application blinks the green LED on the LPSTK.
//
//
//  BOARD SETUP (requires a LaunchPad, for example LAUNCHXL-CC1352R1):
//  - Remove all 11 jumpers on the pin row between the XDS110 and the device on
//    the LaunchPad.
//  - Use the supplied cable to connect JTAG from the LPSTK board to the
//    LaunchPad.
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


// Semaphore used to wait for Sensor Controller task ALERT event
static Semaphore_Struct semScTaskAlert;




void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback




void scTaskAlertCallback(void) {

    // Wake up the OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));

} // scTaskAlertCallback




PIN_Config pLedPinTable[] = {
    Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_State ledPinState;


void taskFxn(UArg a0, UArg a1) {
    PIN_Handle hLedPins;

    // Enable LED pins
    hLedPins = PIN_open(&ledPinState, pLedPinTable);

    // Initialize the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);

    // Set the Sensor Controller task tick interval to 1 second
    scifStartRtcTicksNow(0x00010000);

    // Configure to trigger interrupt at first result, and start the Sensor Controller's I2C Light
    // Sensor task (not to be confused with OS tasks)
    int lowThreshold  = scifTaskData.i2cLightSensor.cfg.lowThreshold  = 1;
    int highThreshold = scifTaskData.i2cLightSensor.cfg.highThreshold = 0;
    scifStartTasksNbl(BV(SCIF_I2C_LIGHT_SENSOR_TASK_ID));

    // Main loop
    while (1) {

        // Wait for an ALERT callback
        Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

        // Clear the ALERT interrupt source
        scifClearAlertIntSource();

        // The light sensor value is outside of the configured window ...
        uint16_t value = scifTaskData.i2cLightSensor.output.value;
        if (value < lowThreshold) {
            // Below the low threshold, so blink red LED
            PIN_setOutputValue(hLedPins, Board_RLED, Board_LED_ON);
            Task_sleep(10000 / Clock_tickPeriod);
            PIN_setOutputValue(hLedPins, Board_RLED, Board_LED_OFF);
        } else if (value > highThreshold) {
            // Above the high threshold, so blink green LED
            PIN_setOutputValue(hLedPins, Board_GLED, Board_LED_ON);
            Task_sleep(10000 / Clock_tickPeriod);
            PIN_setOutputValue(hLedPins, Board_GLED, Board_LED_OFF);
        }

        // Update the thresholds to +/-100 from the current value, with saturation
        lowThreshold = value - 100;
        if (lowThreshold < 0) lowThreshold = 0;
        scifTaskData.i2cLightSensor.cfg.lowThreshold = lowThreshold;
        highThreshold = value + 100;
        if (highThreshold > 65535) highThreshold = 65535;
        scifTaskData.i2cLightSensor.cfg.highThreshold = highThreshold;

        // Acknowledge the alert event
        scifAckAlertEvents();
    }

} // taskFxn




int main(void) {
    Task_Params taskParams;

    // Initialize the board, with shutdown of external flash
    Board_init();

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
