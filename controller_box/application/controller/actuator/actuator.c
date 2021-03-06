#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#include <math.h>

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/GPIO.h>

#include "controller/controller.h"
#include "actuator.h"

#define DAY                86400UL  // 24 hours * 60 minutes * 60 seconds

#define HERTZ_120_TIME    833.0
#define MIN_LOAD_TIME     0.2 * HERTZ_120_TIME
#define MAX_LOAD_TIME     0.8 * HERTZ_120_TIME

//private
static Actuator_Dimmable_t isDimmable(Actuator_t *actuator);
static void clockCB(UArg a0);
static Clock_Handle initClock(Actuator_t *actuator,  Clock_Struct *clkStruct);

//Type, Dimmable?, PIN, pClockHandle
void Actuator_init(Actuator_t *actuator, Actuator_Dimmable_t dimmable, Actuator_Type_t type, uint8_t pin,
                   Clock_Handle *clkHandle, Clock_Struct *clkStruct){

    UTCTimeStruct time;
    time.day = 1;
    time.hour = 0;
    time.minutes = 0;
    time.month = 1;
    time.seconds = 0;
    time.year = 2000;

    setDimmable(actuator, dimmable);
    setLevel(actuator, 0);
    setOffSchedule(actuator, &time);
    setOnSchedule(actuator, &time);
    setPin(actuator, pin);
    setState(actuator, (Actuator_State_t) Actuator_OFF);
    setType(actuator, type);
    setLevel(actuator, 0);


    *clkHandle = initClock(actuator, clkStruct);
    actuator->clkHandle = *clkHandle;
    actuator->clkStruct = *clkStruct;

}

void setDimmable(Actuator_t *actuator, Actuator_Dimmable_t dimmble){
    actuator->dimmable = dimmble;
}

//0-100
void setLevel(Actuator_t *actuator, uint8_t level){
    if(level <= 0){
        actuator->level = 0;
    }
    else if(level >= 100){
        actuator->level = 100;
    }
    else{
        actuator->level = level;
    }
}

//Actuator_ON/Actuator_OFF
void toggleState(Actuator_t *actuator){
    GPIO_toggle(actuator->pin);
    actuator->state ^= (Actuator_State_t) 0x01;
}

//Actuator_ON/Actuator_OFF
void setState(Actuator_t *actuator, Actuator_State_t state){
    GPIO_write(actuator->pin, state);
    actuator->state = state;
}

//pin
void setPin(Actuator_t *actuator, uint8_t pin){
    actuator->pin = pin;
}

//type
void setType(Actuator_t *actuator, Actuator_Type_t type){
    actuator->type = type;
}

//set Schedule
void setOnSchedule(Actuator_t *actuator, UTCTimeStruct *schedule){
    memcpy(&actuator->onSchedule, schedule, sizeof(UTCTimeStruct));
}

//set Schedule
void setOffSchedule(Actuator_t *actuator, UTCTimeStruct *schedule){
    memcpy(&actuator->offSchedule, schedule, sizeof(UTCTimeStruct));
}

//Start dim action
void activateActuator(Actuator_t *actuator){
    /* Stop the Reading timer */
    if(Clock_isActive(actuator->clkHandle))
    {
        Clock_stop(actuator->clkHandle);
    }
    if(isDimmable(actuator)){
        //the 5 represents the number of tens of us it takes to finish this logic. Taking it into consideration here
        uint32_t dim_in_ten_usec = (uint32_t) ((((actuator->level)/100.0) * HERTZ_120_TIME) - 3);

        if(dim_in_ten_usec <= MIN_LOAD_TIME){
            setState(actuator, (Actuator_State_t) Actuator_OFF);
        }
        else if(dim_in_ten_usec >= MAX_LOAD_TIME){
            setState(actuator, (Actuator_State_t) Actuator_ON);
        }
        else{
            setState(actuator, Actuator_ON);
            Clock_setTimeout(actuator->clkHandle, dim_in_ten_usec);
            Clock_start(actuator->clkHandle);
        }
    }else{
        //get current time
        UTCTimeStruct currTime = {0};
        UTC_convertUTCTime(&currTime, UTC_getClock());

        //force day year and month to zero
        currTime.day = 0;
        currTime.month = 0;
        currTime.year = 2000;

        //convert to secs
        uint32_t currTime_secs = (uint32_t) UTC_convertUTCSecs(&currTime);
        uint32_t on_time_secs = (uint32_t) UTC_convertUTCSecs(&actuator->onSchedule);
        uint32_t off_time_secs = (uint32_t) UTC_convertUTCSecs(&actuator->offSchedule);
        uint32_t delta_time_secs = 0;

        //TODO: Optimize this logic! IT IS NASTY!

        if(currTime_secs > on_time_secs){
            if(currTime_secs < off_time_secs){
                //Supposed to be Actuator_ON
                //Turn it Actuator_ON
                setState(actuator, (Actuator_State_t) Actuator_ON);
                //Schedule Actuator_OFF Timer
                delta_time_secs = off_time_secs - currTime_secs;
            }
            else{
                //Supposed to be Actuator_OFF
                //Turn it Actuator_OFF
                setState(actuator, (Actuator_State_t) Actuator_OFF);
                //Schedule Actuator_ON Timer
                delta_time_secs = (DAY - currTime_secs) + on_time_secs;
            }
        }
        else if(currTime_secs > off_time_secs){
            //Supposed to be Actuator_OFF
            //Turn it Actuator_OFF
            setState(actuator, (Actuator_State_t) Actuator_OFF);
            //Schedule Actuator_ON Timer
            delta_time_secs = currTime_secs - on_time_secs;
        }
        else{
            //Both Actuator_ON/Actuator_OFF Schedules are in the future
            //I do not know which one comes first
            if(on_time_secs > off_time_secs){
                //Actuator_OFF Comes first
                //Supposed to be Actuator_ON
                //Turn it Actuator_ON
                setState(actuator, (Actuator_State_t) Actuator_ON);
                //Schedule Actuator_OFF Timer
                delta_time_secs = off_time_secs - currTime_secs;
            }
            else{
                //Supposed to be Actuator_OFF
                //Turn it Actuator_OFF
                setState(actuator, (Actuator_State_t) Actuator_OFF);
                //Schedule Actuator_ON Timer
                delta_time_secs = on_time_secs - currTime_secs;
            }
        }

        Clock_setTimeout(actuator->clkHandle, (uint32_t)(delta_time_secs * 100000));
        Clock_start(actuator->clkHandle);
    }
}

static Actuator_Dimmable_t isDimmable(Actuator_t *actuator){
    return actuator->dimmable;
}

static void clockCB(UArg a0){

    Actuator_t *actuator = (Actuator_t *) a0;
    if(isDimmable(actuator)){
        setState(actuator, Actuator_OFF);
    }else{
        setEvent(CONTROLLER_SYNC);
    }

}

static Clock_Handle initClock(Actuator_t *actuator, Clock_Struct *clkStruct){
    Clock_Params clockParams;

    /* Convert clockDuration in milliseconds to ticks. */
    uint32_t clockTicks = 0;

    /* Setup parameters. */
    Clock_Params_init(&clockParams);

    /* Setup argument. */
    clockParams.arg = (UArg) actuator;

    /* If period is 0, this is a one-shot timer. */
    clockParams.period = 0;

    /*
     Starts immediately after construction if true, otherwise wait for a
     call to start.
     */
    clockParams.startFlag = false;

    /*/ Initialize clock instance. */
    Clock_construct(clkStruct, clockCB, clockTicks, &clockParams);
    return Clock_handle(clkStruct);

}


