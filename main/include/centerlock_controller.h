#ifndef CENTERLOCK_CONTROLLER_H
#define CENTERLOCK_CONTROLLER_H

#include <Arduino.h>
#include <constants.h>
#include <odrive.h>
#include <types.h>

// test out git process

class CenterlockController {
public:
    enum State{
        UNHOMED,
        ENGAGED_4WD,
        DISENGAGED_2WD,
        ENGAGING,
        DISENGAGING,
        // WANT_ENGAGE,
        // WANT_DISENGAGE,
        // PRE_ENGAGING,
        // PRE_DISENGAGING,
        // ENGAGE_STEPBACK
    };
    static const u8 SET_TORQUE_SUCCESS = 0;
    static const u8 SET_TORQUE_OUT_LIMIT_SWITCH_ERROR = 1;
    static const u8 SET_TORQUE_CAN_ERROR = 2;

    static const u8 SET_VELOCITY_SUCCCESS = 0;
    static const u8 SET_VELOCITY_IN_LIMIT_SWITCH_ERROR = 1;
    static const u8 SET_VELOCITY_OUT_LIMIT_SWITCH_ERROR = 1;
    static const u8 SET_VELOCITY_CAN_ERROR = 2;

    static const u8 HOME_SUCCESS = 0;
    static const u8 HOME_CAN_ERROR = 1;
    static const u8 HOME_TIMEOUT_ERROR = 2;

    CenterlockController() {}
    centerlock_controller(ODrive *odrive);
    u8 control(u32 timeout_ms);
    u8 set_Velocity(float velocity);
    u8 fork_position(u32 timeout_ms, bool button_input_4WD, bool button_input_2WD);

    static const float ECENTERLOCK_2WD_VEL = 3; 
    static const float ECENTERLOCK_4WD_VEL = 3;

private: 
u32 Odrive_velocity; 

};

#endif // CENTERLOCK_CONTROLLER_H
