//*****************************************************************************
//  SENSOR CONTROLLER STUDIO EXAMPLE: SPI ACCELEROMETER FOR LAUNCHPAD SENSORTAG
//  KIT (LPSTK)
//  Operating system: TI-RTOS
//
//  The Sensor Controller samples SPI accelerometer on the CC1352R LaunchPad
//  SensorTag Kit (LPSTK), and notifies the System CPU application when tilt
//  detection changes, or when an error occurs:
//  - The LaunchPad's green LED is on when the board is tilted along the
//    accelerometer's X-axis
//  - The LaunchPad's red LED is on when the board is tilted along the
//    accelerometer's Y-axis
//  - Both LEDs blink rapidly to indicate accelerometer error (no data ready
//    interrupt for 20 ms)
//
//  To test the error condition:
//  - Connect the accelerometer INT signal to GND
//
//
//  The accelerometer orientation is printed on the board.
//
//
//  BOARD SETUP (requires a LaunchPad, for example LAUNCHXL-CC1352R1):
//  - Remove all 11 jumpers on the pin row between the XDS110 and the device
//    on the LaunchPad.
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
Task_Struct accelerometerTask;
Char accelerometerTaskStack[1024];

// Semaphore used to wait for Sensor Controller task ALERT event
Semaphore_Struct semScTaskAlert;




void scCtrlReadyCallback(void) {

} // scCtrlReadyCallback




void scTaskAlertCallback(void) {

    // Wake up the accelerometer OS task
    Semaphore_post(Semaphore_handle(&semScTaskAlert));

} // scTaskAlertCallback




PIN_Config pLedPinTable[] = {
    Board_RLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    Board_GLED | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
    PIN_TERMINATE
};
PIN_State ledPinState;
PIN_Handle hLedPins;


void accelerometerTaskFxn(UArg a0, UArg a1) {

    // Enable LED pins
    hLedPins = PIN_open(&ledPinState, pLedPinTable);

    // Initialize and start the Sensor Controller
    scifOsalInit();
    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
    scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);
    scifInit(&scifDriverSetup);
    scifStartTasksNbl(BV(SCIF_SPI_ACCELEROMETER_TASK_ID));

    // Main loop
    while (1) {

        // Wait for an ALERT callback
        Semaphore_pend(Semaphore_handle(&semScTaskAlert), BIOS_WAIT_FOREVER);

        // Clear the ALERT interrupt source
        scifClearAlertIntSource();

        // If the accelerometer has failed, indicate this by blinking both LEDs indefinitely
        if (scifTaskData.spiAccelerometer.output.accelError) {
            while (1) {
                PIN_setOutputValue(hLedPins, Board_GLED, 1);
                PIN_setOutputValue(hLedPins, Board_RLED, 1);
                Task_sleep(50000 / Clock_tickPeriod);
                PIN_setOutputValue(hLedPins, Board_GLED, 0);
                PIN_setOutputValue(hLedPins, Board_RLED, 0);
                Task_sleep(50000 / Clock_tickPeriod);
            }

        // Otherwise indicate tilt detection for each axis
        } else {
            PIN_setOutputValue(hLedPins, Board_GLED, scifTaskData.spiAccelerometer.output.xTiltDet);
            PIN_setOutputValue(hLedPins, Board_RLED, scifTaskData.spiAccelerometer.output.yTiltDet);
        }

        // Acknowledge the ALERT event
        scifAckAlertEvents();
    }

} // accelerometerTaskFxn




int main(void) {

    Task_Params taskParams;
    Semaphore_Params semParams;

    // Initialize the board, with shutdown of external flash
    Board_init();

    // Configure the accelerometer OS task (handles Sensor Controller startup and ALERT interrupts)
    Task_Params_init(&taskParams);
    taskParams.stack = accelerometerTaskStack;
    taskParams.stackSize = sizeof(accelerometerTaskStack);
    taskParams.priority = 3;
    Task_construct(&accelerometerTask, accelerometerTaskFxn, &taskParams, NULL);

    // Create the semaphore used to wait for Sensor Controller ALERT events
    Semaphore_Params_init(&semParams);
    semParams.mode = Semaphore_Mode_BINARY;
    Semaphore_construct(&semScTaskAlert, 0, &semParams);

    // Start TI-RTOS
    BIOS_start();
    return 0;

} // main
