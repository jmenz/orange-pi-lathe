#include "t3d_servo.h"

int modbus_03_read(t3d_servo_t *comp, int reg, uint16_t *value) {
    int ret = modbus_read_registers(comp->mb_ctx, reg, MODBUS_READ_REGISTERS_NUM, value);

    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus read error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp->mb_ctx) < 0) {
            modbus_close(comp->mb_ctx);
        }
        return ret;
    }
    return ret;  // Success
}

int modbus_04_read(t3d_servo_t *comp, int reg, uint16_t *value) {
    int ret = modbus_read_input_registers(comp->mb_ctx, reg, MODBUS_READ_REGISTERS_NUM, value);

    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus 04 read error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp->mb_ctx) < 0) {
            modbus_close(comp->mb_ctx);
        }
        return ret;
    }
    return ret;
}


int modbus_06_write(t3d_servo_t *comp, int reg, uint16_t value) {
    int ret = modbus_write_register(comp->mb_ctx, reg, value);

    if (ret < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Modbus write error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp->mb_ctx) < 0) {
            modbus_close(comp->mb_ctx);
        }
        return ret;
    }
    return ret;
}

int init_modbus(t3d_servo_t *comp) {

    char *device = find_serial_device();
    if (device == NULL) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: No Modbus USB device found!");
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "t3d_servo: Using Modbus device %s", device);

    // Initialize Modbus connection
    comp->mb_ctx = modbus_new_rtu(
        device,
        t3d_modbus_params.baud,
        t3d_modbus_params.parity,
        t3d_modbus_params.data_bit,
        t3d_modbus_params.stop_bit
    );

    if (!comp->mb_ctx) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Failed to create Modbus context");
        return -1;
    }

    modbus_set_slave(comp->mb_ctx, t3d_modbus_params.slave);  // Use configured slave number
    if (modbus_connect(comp->mb_ctx) == -1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "t3d_servo: Failed to connect to Modbus device");
        return -1;
    }

    return 0;
}


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