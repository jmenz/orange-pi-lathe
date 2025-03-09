#include "t3d_servo.h"

extern t3d_servo_t *comp_instance;
extern int comp_id;

int init_hal_component() {
    // Initialize HAL component
    comp_id = hal_init("t3d_servo");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Initialization failed\n");
        return -1;
    }

    // Allocate HAL memory for component
    comp_instance = hal_malloc(sizeof(t3d_servo_t));
    if (!comp_instance) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: HAL memory allocation failed");
        return -1;
    }
    
    comp_instance->last_speed = 0;
    comp_instance->last_control = 0;

    if (init_hal_pins(comp_instance) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: hal pin creation failed");
        return -1;
    }

    return 0;
}

int init_hal_pins(t3d_servo_t *comp) {
    int retval;

    // Create HAL Pins
    retval = hal_pin_float_new("t3d_servo.spindle-speed", HAL_IN, &(comp->spindle_speed), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.on", HAL_IN, &(comp->on), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.hold-motor", HAL_IN, &(comp->hold_motor), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.forward", HAL_IN, &(comp->forward), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.reverse", HAL_IN, &(comp->reverse), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_s32_new("t3d_servo.alarm-code", HAL_OUT, &(comp->alarm_code), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.alarm", HAL_OUT, &(comp->alarm_flag), comp_id);
    if (retval < 0) return retval;

    // Initialize Values
    *(comp->spindle_speed) = 0;
    *(comp->on) = 0;
    *(comp->forward) = 0;
    *(comp->reverse) = 0;
    *(comp->alarm_code) = 0;
    *(comp->alarm_flag) = 0;

    return 0;
}