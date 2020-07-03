#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/drivers/GPIO.h>
#include "smsgs.h"

#ifdef __cplusplus
extern "C"
{
#endif

static void copyActuator(Sensor_actuator_t *actuator, Actuator_t *act);
static void setEvent(uint16_t eventMask);
static void clearEvent(uint16_t eventMask);
void zeroCrossCB(uint_least8_t index);
void runTempPIDTimeoutCallback(UArg a0);
void runHumPIDTimeoutCallback(UArg a0);
void runCo2PIDTimeoutCallback(UArg a0);
static void initializeTempPIDClock(void);
static void initializeHumPIDClock(void);
static void initializeCO2Clock(void);
static void light_init(void);
static void heat_init(void);
static void humidifier_init(void);
static void fan_init(void);
static void init_controller_timers();
static void init_controller_actuators();
static void init_zc();
extern void setTemp(float temp);
extern void setHum(float hum);
extern void setCo2(float co2);
extern void setSetTemp(float temp);
extern void setSetHum(float hum);
extern void setSetCo2(float co2);

extern float getSetTemp(void);
extern float getSetHum(void);
extern float getSetCo2(void);
extern float getTemp(void);
extern float getHum(void);
extern float getCo2(void);
extern void getActuators(Sensor_actuator_t *pActuators);

extern void controller_init(void *evntHandle);
extern void controller_processEvents(void);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */
