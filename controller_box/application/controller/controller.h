#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C"
{
#endif

//starts timer for Dimmable actuators
zeroCrossCB

//runs periodically and updates the level value, based on current values
tempPID
humPID
co2PID

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */
