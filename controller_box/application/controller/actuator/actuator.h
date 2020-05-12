#ifndef ACTUATOR_H
#define ACTUATOR_H

#include <ti/sysbios/knl/Clock.h>

#include <controller/time/utc_clock.h>
#include <controller/time/time_clock.h>

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
    NOT_DIMMABLE,
    DIMMABLE
} Actuator_Dimmable_t;

typedef enum
{
    HUMIDITY      ,
    TEMPERATURE   ,
    LIGHT         ,
    ACCELEROMETER ,
    HALL_EFFECT   ,
    AIR_QUALITY
} Actuator_Type_t;

typedef enum
{
    Actuator_OFF,
    Actuator_ON
} Actuator_State_t;

typedef struct
{
    Actuator_Dimmable_t   dimmable;
    uint8_t             pin;
    Actuator_State_t      state;
    Actuator_Type_t       type;
    uint8_t             level;
    UTCTimeStruct       onSchedule;
    UTCTimeStruct       offSchedule;
    Clock_Handle        clkHandle;
    Clock_Struct        clkStruct;
} Actuator_t;

/*********************************************************************
 * FUNCTIONS
 */

/*
 * @fn      Actuator_init
 *
 * @brief   Initialize the actuator struct.
 *
 * @param   None.
 *
 * @return  None.
 */
//Type, Dimmable?, PIN, pClockHandle
extern void Actuator_init(Actuator_t *actuator, Actuator_Dimmable_t dimmable, Actuator_Type_t type, uint8_t pin,
                          Clock_Handle *clkHandle, Clock_Struct *clkStruct);
extern void setDimmable(Actuator_t *actuator, Actuator_Dimmable_t dimmble);
//0-100
extern void setLevel(Actuator_t *actuator, uint8_t level);
extern void toggleState(Actuator_t *actuator);
//ON/OFF
extern void setState(Actuator_t *actuator, Actuator_State_t state);
//pin
extern void setPin(Actuator_t *actuator, uint8_t pin);
//type
extern void setType(Actuator_t *actuator, Actuator_Type_t type);
//set Schedule
extern void setOnSchedule(Actuator_t *actuator, UTCTimeStruct schedule);
//set Schedule
extern void setOffSchedule(Actuator_t *actuator, UTCTimeStruct schedule);
//Start regular action
extern void activateActuator(Actuator_t *actuator);

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ACTUATOR_H */
