#include "t3d_servo.h"

extern t3d_servo_t *comp_instance;

int init_hal_pins(int *comp_id){
    int retval;

    // Create HAL Pins
    retval = hal_pin_float_new("t3d_servo.spindle-speed", HAL_IN, &(comp_instance->spindle_speed), *comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.on", HAL_IN, &(comp_instance->on), *comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.hold-motor", HAL_IN, &(comp_instance->hold_motor), *comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.forward", HAL_IN, &(comp_instance->forward), *comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.reverse", HAL_IN, &(comp_instance->reverse), *comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_s32_new("t3d_servo.alarm-code", HAL_OUT, &(comp_instance->alarm_code), *comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.alarm", HAL_OUT, &(comp_instance->alarm_flag), *comp_id);
    if (retval < 0) return retval;

    // Initialize Values
    *(comp_instance->spindle_speed) = 0;
    *(comp_instance->on) = 0;
    *(comp_instance->forward) = 0;
    *(comp_instance->reverse) = 0;
    *(comp_instance->alarm_code) = 0;
    *(comp_instance->alarm_flag) = 0;

    return 0;
}