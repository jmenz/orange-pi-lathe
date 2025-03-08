#include "t3d_servo.h"


static t3d_servo_t *comp_instance;  // Component instance
static int comp_id; // Store the component ID

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

    retval = hal_pin_s32_new("t3d_servo.alarm-code", HAL_OUT, &(comp_instance->alarm_code), comp_id);
    if (retval < 0) return retval;

    retval = hal_pin_bit_new("t3d_servo.alarm", HAL_OUT, &(comp_instance->alarm_flag), comp_id);
    if (retval < 0) return retval;

    // Initialize Values
    *(comp_instance->spindle_speed) = 0;
    *(comp_instance->on) = 0;
    *(comp_instance->forward) = 0;
    *(comp_instance->reverse) = 0;
    *(comp_instance->alarm_code) = 0;
    *(comp_instance->alarm_flag) = 0;

    comp_instance->last_speed = 0;
    comp_instance->last_control = 0;

    char *device = find_serial_device();
    if (device == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: No Modbus USB device found!");
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "t3d_servo: Using Modbus device %s", device);

    // Initialize Modbus connection
    comp_instance->mb_ctx = modbus_new_rtu(
        device,
        t3d_modbus_params.baud,
        t3d_modbus_params.parity,
        t3d_modbus_params.data_bit,
        t3d_modbus_params.stop_bit
    );

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

    update_control(comp);
    update_speed(comp);

    rtapi_u64 current_time = rtapi_get_time();

    // Read Modbus every 1 second (1,000,000,000 nanoseconds)
    if ((current_time - comp->last_modbus_read_time) >= 1000000000) {
        read_alarm(comp);
        comp->last_modbus_read_time = current_time;
    }
}

void update_speed(t3d_servo_t *comp) {
    // Only send speed if it changed (MODBUS_REG_RPM, Function 06)
    if (*(comp->spindle_speed) != comp->last_speed) {
        uint16_t speed_val = *(comp->spindle_speed);

        if (modbus_06_write(comp->mb_ctx, MODBUS_REG_RPM, speed_val) >= 0) {
            comp->last_speed = *(comp->spindle_speed);
        }
    }
}

void update_control(t3d_servo_t *comp) {
    // Determine control command
    uint16_t control_val = t3d_servo_control.stop;  // Default to STOP
    if (*(comp->on)) {
        if (*(comp->forward) && !*(comp->reverse)) {
            control_val = t3d_servo_control.forward;  // FORWARD
        } else if (*(comp->reverse) && !*(comp->forward)) {
            control_val = t3d_servo_control.reverse;  // REVERSE
        }
    }

    // Only send control command if it changed
    if (control_val != comp->last_control) {
        if (modbus_06_write(comp->mb_ctx, MODBUS_REG_CONTROL, control_val) >= 0) {
            comp->last_control = control_val;
        }
    }
}

void read_alarm(t3d_servo_t *comp) {
    
    uint16_t alarm_code;

    if (modbus_04_read(comp->mb_ctx, MODBUS_REG_ALARM, &alarm_code) >= 0) {
        *comp->alarm_code = alarm_code;
        *comp->alarm_flag = (alarm_code > 0) ? 1 : 0;
    }
}

int modbus_check_connection(t3d_servo_t *comp) {
    uint16_t test_reg;
    int status = modbus_read_registers(comp->mb_ctx, 76, 1, &test_reg);
    
    if (status == -1) {  // Modbus error
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus connection lost. Attempting to reconnect...");
        
        modbus_close(comp->mb_ctx);
        modbus_free(comp->mb_ctx);
        
        char *device = find_serial_device();
        if (device == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: No Modbus USB device found.");
            return -1;
        }
        
        comp->mb_ctx = modbus_new_rtu(device, 19200, 'E', 8, 1);
        if (comp->mb_ctx == NULL) {
            rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Failed to create Modbus context.");
            return -1;
        }

        modbus_set_slave(comp->mb_ctx, 1);
        if (modbus_connect(comp->mb_ctx) == -1) {
            rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Reconnection failed.");
            modbus_free(comp->mb_ctx);
            return -1;
        }

        rtapi_print_msg(RTAPI_MSG_INFO, "t3d_servo: Reconnected to Modbus device.");
    }
    
    return 0; // Connection OK
}

int modbus_03_read(modbus_t *mb_ctx, int reg, uint16_t *value) {
    int ret = modbus_read_registers(comp_instance->mb_ctx, reg, 1, value);

    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus read error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp_instance->mb_ctx) < 0) {
            modbus_close(comp_instance->mb_ctx);
        }
        return ret;
    }
    return ret;  // Success
}

int modbus_04_read(modbus_t *mb_ctx, int reg, uint16_t *value) {
    int ret = modbus_read_input_registers(mb_ctx, reg, 1, value);

    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus 04 read error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(mb_ctx) < 0) {
            modbus_close(mb_ctx);
        }
        return ret;
    }
    return ret;  // Success
}


int modbus_06_write(modbus_t *mb_ctx, int reg, uint16_t value) {
    int ret = modbus_write_register(mb_ctx, reg, value);

    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus write error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp_instance->mb_ctx) < 0) {
            modbus_close(comp_instance->mb_ctx);
        }
        return ret;
    }
    return ret;
}


// Entry point for the component
int rtapi_app_main(void) {
    return init();  // Calls the existing init function
}

// Cleanup function
void rtapi_app_exit(void) {
    if (comp_instance && comp_instance->mb_ctx) {
        modbus_06_write(comp_instance->mb_ctx, MODBUS_REG_CONTROL, t3d_servo_control.off);
        modbus_close(comp_instance->mb_ctx);
        modbus_free(comp_instance->mb_ctx);
    }
    hal_exit(comp_id);
}
