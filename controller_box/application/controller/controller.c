#include "controller.h"
#include "actuator/actuator.h"
#include <controller/time/utc_clock.h>
#include <controller/time/time_clock.h>

#include "util_timer.h"
#include "mac_util.h"
#include "ssf.h"

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>


#include "ti_drivers_config.h"
#include "sensor.h"
/******************************************************************************
 Constants and definitions
 *****************************************************************************/




#define TEMP_PID_TIMEOUT_VALUE  5000
#define HUM_PID_TIMEOUT_VALUE   5000
#define CO2_PID_TIMEOUT_VALUE   5000
#define SYNC_TIMEOUT_VALUE      60000
#define TOTAL_ACTUATORS 4

static Semaphore_Handle applicationSem;
static uint16_t controllerEvents;

static PID_t tempPID;
static PID_t humPID;
static PID_t co2PID;

static Actuator_t light;
static Actuator_t heat;
static Actuator_t humidifier;
static Actuator_t solenoid;

static Control_t control;
static Actuator_t * activeActuators[TOTAL_ACTUATORS];

static Clock_Handle lightClkHandle;
static Clock_Handle heatClkHandle;
static Clock_Handle humidifierClkHandle;
static Clock_Handle solenoidClkHandle;

static Clock_Struct lightClkStruct;
static Clock_Struct heatClkStruct;
static Clock_Struct humidifierClkStruct;
static Clock_Struct solenoidClkStruct;

static Clock_Handle tempPIDClkHandle;
static Clock_Handle humPIDClkHandle;
static Clock_Handle co2PIDClkHandle;
static Clock_Handle syncClkHandle;

static Clock_Struct tempPIDClkStruct;
static Clock_Struct humPIDClkStruct;
static Clock_Struct co2PIDClkStruct;
static Clock_Struct syncClkStruct;

void setEvent(uint16_t eventMask)
{
    controllerEvents |= eventMask;
    /* Wake up the application thread when it waits for keys event */
    Semaphore_post(applicationSem);
}

static void clearEvent(uint16_t eventMask)
{
    controllerEvents ^= eventMask;

    /* Wake up the application thread when it waits for keys event */
    Semaphore_post(applicationSem);
}

void zeroCrossCB(uint_least8_t index){
    activateActuator(&heat);
}

void runTempPIDTimeoutCallback(UArg a0){
    setEvent(a0);
}

void runHumPIDTimeoutCallback(UArg a0){
    setEvent(a0);
}

void runCo2PIDTimeoutCallback(UArg a0){
    setEvent(a0);
}

void runSyncTimeoutCallback(UArg a0){
    sendSyncReq();
}

static void initializeTempPIDClock(void)
{
    /* Initialize the timers needed for this application */
    tempPIDClkHandle = Timer_construct(&tempPIDClkStruct,
                                           runTempPIDTimeoutCallback,
                                           TEMP_PID_TIMEOUT_VALUE,
                                           TEMP_PID_TIMEOUT_VALUE,
                                           false,
                                           CONTROLLER_TEMP_PID);
}

static void initializeHumPIDClock(void){
    /* Initialize the timers needed for this application */
    humPIDClkHandle = Timer_construct(&humPIDClkStruct,
                                          runHumPIDTimeoutCallback,
                                          HUM_PID_TIMEOUT_VALUE,
                                          HUM_PID_TIMEOUT_VALUE,
                                          false,
                                          CONTROLLER_HUM_PID);
}

static void initializeCO2Clock(void){
    /* Initialize the timers needed for this application */
    co2PIDClkHandle = Timer_construct(&co2PIDClkStruct,
                                          runCo2PIDTimeoutCallback,
                                          CO2_PID_TIMEOUT_VALUE,
                                          CO2_PID_TIMEOUT_VALUE,
                                          false,
                                          CONTROLLER_CO2_PID);
}
static void initializeSyncClock(void){
    /* Initialize the timers needed for this application */
    syncClkHandle = Timer_construct(&syncClkStruct,
                                          runSyncTimeoutCallback,
                                          SYNC_TIMEOUT_VALUE,
                                          SYNC_TIMEOUT_VALUE,
                                          false,
                                          CONTROLLER_SYNC);
}
static void light_init(void){
    Actuator_init(&light, NOT_DIMMABLE, LIGHT, CONFIG_GPIO_LIGHT, &lightClkHandle, &lightClkStruct);
}

static void heat_init(void){
    Actuator_init(&heat, DIMMABLE, TEMPERATURE, CONFIG_GPIO_HEAT, &heatClkHandle, &heatClkStruct);
}

static void humidifier_init(void){
    Actuator_init(&humidifier, NOT_DIMMABLE, HUMIDITY, CONFIG_GPIO_HUM, &humidifierClkHandle, &humidifierClkStruct);
}

static void solenoid_init(void){
    Actuator_init(&solenoid, NOT_DIMMABLE, AIR_QUALITY, CONFIG_GPIO_SOLENOID, &solenoidClkHandle, &solenoidClkStruct);
}

static void init_controller_timers()
{
    initializeTempPIDClock();
    initializeHumPIDClock();
    initializeCO2Clock();
    initializeSyncClock();
}

static void init_controller_actuators()
{
    light_init();
    heat_init();
    humidifier_init();
    solenoid_init();
    activeActuators[0] = &light;
    activeActuators[1] = &heat;
    activeActuators[2] = &humidifier;
    activeActuators[3] = &solenoid;
}

static void init_zc()
{
    GPIO_setCallback(CONFIG_GPIO_ZC, zeroCrossCB);
    GPIO_enableInt(CONFIG_GPIO_ZC);
}

void setTemp(float temp){
    control.temperature = temp;
}

void setHum(float hum){
    control.humidity = hum;
}

void setCo2(float co2){
    control.co2 = co2;
}

void setSetTemp(float temp){
    tempPID.setPoint = temp;
}

void setSetHum(float hum){
    humPID.setPoint = hum;
}

void setSetCo2(float co2){
    co2PID.setPoint = co2;
}

extern void setSchedule(uint8_t* pBuf){
    UTCTime On_sec;
    UTCTime Off_Sec;

    memcpy(&On_sec, pBuf, 4);
    pBuf += 4;
    memcpy(&Off_Sec, pBuf, 4);
    pBuf += 4;

    UTCTimeStruct On;
    UTCTimeStruct Off;

    UTC_convertUTCTime(&On, On_sec);
    UTC_convertUTCTime(&Off, Off_Sec);

    setOnSchedule(&light, &On);
    setOffSchedule(&light, &Off);

    setEvent(CONTROLLER_SYNC);
}

extern float getSetTemp(void){
    return tempPID.setPoint;
}
extern float getSetHum(void){
    return humPID.setPoint;
}
extern float getSetCo2(void){
    return co2PID.setPoint;
}
extern float getTemp(void){
    return control.temperature;
}
extern float getHum(void){
    return control.humidity;
}
extern float getCo2(void){
    return control.co2;
}

static void copyActuator(Sensor_actuator_t *actuator, Actuator_t *act){
    actuator->dimmable = act->dimmable;
    actuator->level = act->level;
    actuator->state = act->state;
    actuator->type = act->type;
}

extern void getActuators(Sensor_actuator_t *pActuators){
    Sensor_actuator_t tempList[TOTAL_ACTUATORS];
    memset(&tempList, 0, sizeof(Sensor_actuator_t)*TOTAL_ACTUATORS);
    copyActuator(&tempList[0], &light);
    copyActuator(&tempList[1], &heat);
    copyActuator(&tempList[2], &humidifier);
    copyActuator(&tempList[3], &solenoid);
    memcpy(pActuators, tempList, sizeof(Sensor_actuator_t)*TOTAL_ACTUATORS);
}


void controller_init(void *evntHandle){
    applicationSem = evntHandle;
    GPIO_init();
    //Configure ZC input pin
    init_zc();
    init_controller_timers();
    init_controller_actuators();
    setEvent(CONTROLLER_START);
}

void controller_processEvents(void){
    Ssf_displaySchedule(&light);
    Ssf_displayControl(control, co2PID.setPoint, tempPID.setPoint, humPID.setPoint );
    Ssf_displayActuator(activeActuators, TOTAL_ACTUATORS);
    Ssf_displayTime();
    if(!controllerEvents)
    {
        return;
    }
    if(controllerEvents & CONTROLLER_START)
    {
        UTC_init();
        Timer_start(&syncClkStruct);
        Timer_start(&tempPIDClkStruct);
        Timer_start(&humPIDClkStruct);
        Timer_start(&co2PIDClkStruct);

        tempPID.eT2 = 0;
        tempPID.eT1 = 0;
        tempPID.eT0 = 0;
        tempPID.cT2 = 0;
        tempPID.eT1 = 0;

        control.co2 = 0;
        control.temperature = 0;
        control.humidity = 0;

        tempPID.setPoint = 70;
        humPID.setPoint = 50;
        co2PID.setPoint = 400;

        clearEvent(CONTROLLER_START);
    }
    if(controllerEvents & CONTROLLER_SYNC)
    {
        activateActuator(&light);

        clearEvent(CONTROLLER_SYNC);
    }
    if(controllerEvents & CONTROLLER_TEMP_PID)
    {
        tempPID.eT2 = tempPID.eT1;
        tempPID.eT1 = tempPID.eT0;
        tempPID.eT0 = (int16_t) (tempPID.setPoint - control.temperature);

        tempPID.cT2 = tempPID.cT1;
        tempPID.cT1 = tempPID.cT0;

        tempPID.cT0 = (int16_t) (tempPID.eT1 - (0.974*tempPID.eT2) + (1.904*tempPID.cT1) - (0.904*tempPID.cT2));

        if(tempPID.cT0 >= 80){
            tempPID.cT0 = 100;
        }else if(tempPID.cT0 <= 20){
            tempPID.cT0 = 0;
        }

        setLevel(&heat,(uint8_t) tempPID.cT0);

        clearEvent(CONTROLLER_TEMP_PID);
    }
    if(controllerEvents & CONTROLLER_HUM_PID)
    {

        humPID.cT0 = (int16_t) humPID.setPoint - 3;
        humPID.cT1 = (uint16_t) humPID.setPoint + 3;

        if(control.humidity <= humPID.cT0){
            setState(&humidifier, Actuator_ON);
        }
        else if(control.humidity >= humPID.cT1){
            setState(&humidifier, Actuator_OFF);
        }

        clearEvent(CONTROLLER_HUM_PID);
    }
    if(controllerEvents & CONTROLLER_CO2_PID)
    {
        co2PID.cT0 = (int16_t) co2PID.setPoint - 100;
        co2PID.cT1 = (int16_t) co2PID.setPoint + 100;

        if(control.co2 <= co2PID.cT0){
            setState(&solenoid, Actuator_ON);
        }
        else if(control.co2 >= co2PID.cT1){
            setState(&solenoid, Actuator_OFF);
        }

        clearEvent(CONTROLLER_CO2_PID);
    }
}
