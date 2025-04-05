#include "t3d_servo.h"

t3d_servo_t *init_hal_component(int *comp_id) {
    // Initialize HAL component
    *comp_id = hal_init("t3d_servo");
    if (*comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "T3D_SERVO: Initialization failed\n");
        return NULL;
    }

    t3d_servo_t *comp = hal_malloc(sizeof(t3d_servo_t));
    if (!comp) {
        rtapi_print_msg(RTAPI_MSG_ERR, "T3D_SERVO: hal_malloc failed! Exiting...\n");
        return NULL;
    }

    comp->last_speed = 0;
    comp->last_command = 0;
    comp->last_on_status = 0;
    comp->modbus_reconnect_attempts = 0;

    // Initialize HAL Pins
    if (init_hal_pins(comp, *comp_id) < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "T3D_SERVO: Failed to initialize HAL pins\n");
        return NULL;
    }

    return comp;  // Return the allocated instance
}

int init_hal_pins(t3d_servo_t *comp, int comp_id) {
    int retval;

    // Create HAL Pins
    retval = hal_pin_bit_new("t3d_servo.enable", HAL_IN, &(comp->enable), comp_id);
    if (retval < 0) return retval;

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

    retval = hal_pin_u32_new("t3d_servo.motor-release-delay", HAL_IN, &(comp->motor_release_delay), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.reset-alarm", HAL_IN, &(comp->reset_alarm), comp_id);
    if (retval < 0) return retval;

    retval = hal_param_u32_new("t3d_servo.speed-limit", HAL_RW , &(comp->speed_limit), comp_id);
    if (retval < 0) return retval;

    // Initialize Values
    *(comp->enable) = 0;
    *(comp->spindle_speed) = 0;
    *(comp->on) = 0;
    *(comp->hold_motor) = 0;
    *(comp->forward) = 0;
    *(comp->reverse) = 0;
    *(comp->alarm_code) = 0;
    *(comp->motor_release_delay) = 500;
    *(comp->reset_alarm) = 0;

    comp->speed_limit = 2500.0;

    return 0;
}