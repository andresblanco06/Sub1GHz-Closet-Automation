#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/GPIO.h>

#include "controller.h"
#include "actuator/actuator.h"
#include <controller/time/utc_clock.h>
#include <controller/time/time_clock.h>

/*!
 Initialize the .

 Public function defined in ssf.h
 */
void initializeZeroCrossClock(void)
{
    /* Initialize the timers needed for this application */
    scanBackoffClkHandle = Timer_construct(&scanBackoffClkStruct,
                                           processScanBackoffTimeoutCallback,
                                           SCAN_BACKOFF_TIMEOUT_VALUE,
                                           0,
                                           false,
                                           0);
}



/*!
 * @brief       Main task function
 *
 * @param       a0 -
 * @param       a1 -
 */
Void controllerTaskFxn(UArg a0, UArg a1)
{
    GPIO_init();


    /* Kick off application - Forever loop */
    while(1)
    {
        Sensor_process();
    }
}
