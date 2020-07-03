#include "controller.h"
#include "actuator/actuator.h"
#include <controller/time/utc_clock.h>
#include <controller/time/time_clock.h>

#include "util_timer.h"
#include "mac_util.h"

#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

#include "ti_drivers_config.h"
#include "sensor.h"
/******************************************************************************
 Constants and definitions
 *****************************************************************************/
/* controller Events */
typedef enum
{
    CONTROLLER_START                 = Event_Id_00,
    CONTROLLER_SYNC                  = Event_Id_01,
    CONTROLLER_TEMP_PID              = Event_Id_02,
    CONTROLLER_HUM_PID               = Event_Id_03,
    CONTROLLER_CO2_PID               = Event_Id_04,
} controller_Events;

typedef struct
{
    float    p;
    float    i;
    float    d;
    float    eT2;
    float    eT1;
    float    eT0;
    float    cT2;
    float    cT1;
    float    cT0;
    float    setPoint;
} PID_t;

struct
{
    float temperature;
    float humidity;
    float co2;
} control;

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
static Actuator_t fan;

static Clock_Handle lightClkHandle;
static Clock_Handle heatClkHandle;
static Clock_Handle humidifierClkHandle;
static Clock_Handle fanClkHandle;

static Clock_Struct lightClkStruct;
static Clock_Struct heatClkStruct;
static Clock_Struct humidifierClkStruct;
static Clock_Struct fanClkStruct;

static Clock_Handle tempPIDClkHandle;
static Clock_Handle humPIDClkHandle;
static Clock_Handle co2PIDClkHandle;
static Clock_Handle syncClkHandle;

static Clock_Struct tempPIDClkStruct;
static Clock_Struct humPIDClkStruct;
static Clock_Struct co2PIDClkStruct;
static Clock_Struct syncClkStruct;

static void setEvent(uint16_t eventMask)
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
//    activateActuator(&light);
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
    setEvent(a0);
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

static void fan_init(void){
    Actuator_init(&fan, NOT_DIMMABLE, AIR_QUALITY, CONFIG_GPIO_FAN, &fanClkHandle, &fanClkStruct);
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
    fan_init();
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
    copyActuator(&tempList[3], &fan);
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
        clearEvent(CONTROLLER_SYNC);
    }
    if(controllerEvents & CONTROLLER_TEMP_PID)
    {
        tempPID.eT2 = tempPID.eT1;
        tempPID.eT1 = tempPID.eT0;
        tempPID.eT0 = tempPID.setPoint - control.temperature;

        tempPID.cT2 = tempPID.cT1;
        tempPID.eT1 = tempPID.cT0;

        tempPID.eT0 = tempPID.eT1 - (0.974*tempPID.eT2) + (1.904*tempPID.cT1) - (0.904*tempPID.cT2);

        if(tempPID.eT0 > 100){
            tempPID.eT0 = 100;
        }else if(tempPID.eT0 <= 0){
            tempPID.eT0 = 0;
        }

        setLevel(&heat, tempPID.eT0);

        clearEvent(CONTROLLER_TEMP_PID);
    }
    if(controllerEvents & CONTROLLER_HUM_PID)
    {

        humPID.eT0 = humPID.setPoint - 3;
        humPID.eT1 = humPID.setPoint + 3;

        if(control.humidity <= humPID.eT0){
            setState(&humidifier, Actuator_ON);
        }
        else if(control.humidity >= humPID.eT1){
            setState(&humidifier, Actuator_OFF);
        }

        clearEvent(CONTROLLER_HUM_PID);
    }
    if(controllerEvents & CONTROLLER_CO2_PID)
    {
        co2PID.eT0 = co2PID.setPoint - 100;
        co2PID.eT1 = co2PID.setPoint + 100;

        if(control.co2 <= co2PID.eT0){
            setState(&fan, Actuator_ON);
        }
        else if(control.co2 >= co2PID.eT1){
            setState(&fan, Actuator_OFF);
        }

        clearEvent(CONTROLLER_CO2_PID);
    }
}
