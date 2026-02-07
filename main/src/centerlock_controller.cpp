#include "centerlock_controller.h"
#include <constants.h>
#include <odrive.h>

uint32_t out_pin_ =  1;
uint32_t in_pin_ = 2;

// make centerlock limit switches 
CenterlockLimitSwitch centerlocklimitswitch(out_pin_, in_pin_);

Centerlock_controller::Centerlock_controller(ODrive *odrive) : odrive(odrive), current_state(UNHOMED), engage(false), num_tries(0), cycles_since_stopped(0) {}

// Call homing sequence when first turned on
u8 Centerlock_controller::home(u32 timeout_ms){

    // push all the way out 
    if(odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL)){
        return HOME_CAN_ERROR;
    }

    u32 start_time = millis();
    Serial.printf("Engaging...");   

    // set velocity on Odrive for the centerlock shifting
    set_velocity(ECENTERLOCK_HOME_VEL);
    float cur_pos = 0;

    return HOME_SUCCESS;
}   

// Call centerlock controller
u8 Centerlock_controller::control(u32 timeout_ms, bool button_input_4WD, bool button_input_2WD){

    u32 start = millis();
    float cur_pos = 0;

    if(button_input_2WD){
        while(!centerlocklimitswitch.is_outbound()){
            if(millis() - start > timeout_ms){
                return CONTROL_TIMEOUT;
            }

            odrive->request_nonstand_pos_rel();
            set_velocity(ECENTERLOCK_2WD_VEL);
            delay(50);

            cur_pos = odrive->get_pos_rel();
            Serial.printf("Shifting to 2WD: %f\n", cur_pos);
        }
        return CONTROL_SUCCESS;
    }

    if(button_input_4WD){
        while(!centerlocklimitswitch.is_inbound()){
            if(millis() - start > timeout_ms){
                return CONTROL_TIMEOUT;
            }

            odrive->request_nonstand_pos_rel();
            set_velocity(ECENTERLOCK_4WD_VEL);
            delay(50);

            cur_pos = odrive->get_pos_rel();
            Serial.printf("Engaging 4WD: %f\n", cur_pos);
        }
        return CONTROL_SUCCESS;
    }

    return CONTROL_IDLE;
}

u8 Centerlock_controller::set_velocity(float velocity) {
  if (odrive->get_axis_state() == ODrive::AXIS_STATE_IDLE) {
    odrive->set_axis_state(ODrive::AXIS_STATE_CLOSED_LOOP_CONTROL);
  }

  if (odrive->set_controller_mode(ODrive::CONTROL_MODE_VELOCITY_CONTROL,
                                  ODrive::INPUT_MODE_VEL_RAMP) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  velocity = CLAMP(velocity, -ODRIVE_VEL_LIMIT, ODRIVE_VEL_LIMIT);
  if (odrive->set_input_vel(velocity, 0) != 0) {
    return SET_VELOCITY_CAN_ERROR;
  }

  velocity_mode = true;

  return SET_VELOCITY_SUCCCESS;
}
