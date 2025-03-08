#include "t3d_servo.h"


static t3d_servo_t *comp_instance;  // Component instance
static int comp_id; // Store the component ID

// 🔹 Define Register Addresses
#define MODBUS_REG_RPM      76      // RPM Setpoint Register
#define MODBUS_REG_CONTROL  4112    // Control Command Register
#define MODBUS_REG_ALARM    26      // Error Code Register

// 🔹 Define Control Command Values (from HAL MUX configuration)
typedef struct {
    uint16_t stop;
    uint16_t forward;
    uint16_t reverse;
} t3d_servo_control_t;

static const t3d_servo_control_t t3d_servo_control = {
    .stop    = 4660,  // STOP Command
    .forward = 8738,  // FORWARD Command
    .reverse = 4369   // REVERSE Command
};


char *find_serial_device() {
    static char device_path[256];
    glob_t glob_result;

    // Look for any device inside /dev/serial/by-id/
    if (glob("/dev/serial/by-id/*", 0, NULL, &glob_result) == 0) {
        if (glob_result.gl_pathc > 0) {
            strncpy(device_path, glob_result.gl_pathv[0], sizeof(device_path) - 1);
            globfree(&glob_result);
            return device_path;
        }
        globfree(&glob_result);
    }

    return NULL;  // No matching device found
}

int init(void) {
    int retval;

    // Initialize HAL component
    comp_id = hal_init("t3d_servo");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: HAL initialization failed");
        return -1;
    }

    // Allocate HAL memory for component
    comp_instance = hal_malloc(sizeof(t3d_servo_t));
    if (!comp_instance) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: HAL memory allocation failed");
        return -1;
    }

    // Create HAL Pins
    retval = hal_pin_float_new("t3d_servo.spindle-speed", HAL_IN, &(comp_instance->spindle_speed), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.on", HAL_IN, &(comp_instance->on), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.forward", HAL_IN, &(comp_instance->forward), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.reverse", HAL_IN, &(comp_instance->reverse), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_s32_new("t3d_servo.error-code", HAL_OUT, &(comp_instance->error_code), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.error-flag", HAL_OUT, &(comp_instance->error_flag), comp_id);
    if (retval < 0) return retval;

    // Initialize Values
    *(comp_instance->spindle_speed) = 0;
    *(comp_instance->on) = 0;
    *(comp_instance->forward) = 0;
    *(comp_instance->reverse) = 0;
    *(comp_instance->error_code) = 0;
    *(comp_instance->error_flag) = 0;

    comp_instance->last_speed = 0;
    comp_instance->last_control = 0;

    char *device = find_serial_device();
    if (device == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: No Modbus USB device found!");
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "t3d_servo: Using Modbus device %s", device);

    // Initialize Modbus connection
    comp_instance->mb_ctx = modbus_new_rtu(device, 19200, 'E', 8, 1);
    if (!comp_instance->mb_ctx) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Failed to create Modbus context");
        return -1;
    }

    modbus_set_slave(comp_instance->mb_ctx, 1);  // Use configured slave number
    if (modbus_connect(comp_instance->mb_ctx) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Failed to connect to Modbus device");
        return -1;
    }

    // Register Function
    retval = hal_export_funct("t3d_servo.update", update, comp_instance, 1, 0, comp_id);
    if (retval < 0) return retval;

    // Finalize HAL component setup
    hal_ready(comp_id);

    rtapi_print_msg(RTAPI_MSG_INFO, "t3d_servo: Loaded successfully");
    return 0;
}

void update(void *arg, long period) {
    t3d_servo_t *comp = (t3d_servo_t *)arg;

    int ret;

    // Read Alarm/Error Code (MODBUS_REG_ALARM, Function 04)
    // uint16_t alarm_reg;
    // int ret;
    
    // ret = modbus_read_input_registers(comp->mb_ctx, MODBUS_REG_ALARM, 1, &alarm_reg);

    // if (ret == 1) {
    //     *(comp->error_code) = alarm_reg;
    //     rtapi_print_msg(RTAPI_MSG_INFO, alarm_reg);
    // } else {

    //     rtapi_print_msg(RTAPI_MSG_INFO, ret);
    //     rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Error reading alarm register");
    // }

    // // Set error flag if error_code > 0
    // *(comp->error_flag) = (*(comp->error_code) > 0);

    // Only send speed if it changed (MODBUS_REG_RPM, Function 06)
    if (*(comp->spindle_speed) != comp->last_speed) {
        uint16_t speed_val = *(comp->spindle_speed);
        ret = modbus_write_register(comp->mb_ctx, MODBUS_REG_RPM, speed_val);
        if (ret != 1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Error writing speed to register");
        }
        comp->last_speed = *(comp->spindle_speed);
    }

    // Determine control command
    uint16_t control_val = t3d_servo_control.stop;  // Default to STOP
    if (*(comp->on)) {
        if (*(comp->forward) && !*(comp->reverse)) {
            control_val = t3d_servo_control.forward;  // FORWARD
        } else if (*(comp->reverse) && !*(comp->forward)) {
            control_val = t3d_servo_control.reverse;  // REVERSE
        }
    }

    // Only send control command if it changed (MODBUS_REG_CONTROL, Function 06)
    if (control_val != comp->last_control) {
        if (modbus_write_register(comp->mb_ctx, MODBUS_REG_CONTROL, control_val) != 1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Error writing control to register");
        }
        comp->last_control = control_val;
    }
}


// Define HAL component ID
static int comp_id;

// Entry point for the component
int rtapi_app_main(void) {
    return init();  // Calls the existing init function
}

// Cleanup function
void rtapi_app_exit(void) {
    if (comp_instance && comp_instance->mb_ctx) {
        modbus_close(comp_instance->mb_ctx);
        modbus_free(comp_instance->mb_ctx);
    }
    hal_exit(comp_id);
}
