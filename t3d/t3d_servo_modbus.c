#include "t3d_servo.h"

extern t3d_servo_t *comp_instance; 

int modbus_03_read(int reg, uint16_t *value) {
    int ret = modbus_read_registers(comp_instance->mb_ctx, reg, MODBUS_READ_REGISTERS_NUM, value);

    if (ret < 0) {
        fprintf(stderr, "t3d_servo: Modbus read error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp_instance->mb_ctx) < 0) {
            modbus_close(comp_instance->mb_ctx);
        }
        return ret;
    }
    return ret;  // Success
}

int modbus_04_read(int reg, uint16_t *value) {
    int ret = modbus_read_input_registers(comp_instance->mb_ctx, reg, MODBUS_READ_REGISTERS_NUM, value);

    if (ret < 0) {
        fprintf(stderr, "t3d_servo: Modbus 04 read error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp_instance->mb_ctx) < 0) {
            modbus_close(comp_instance->mb_ctx);
        }
        return ret;
    }
    return ret;
}


int modbus_06_write(int reg, uint16_t value) {
    int ret = modbus_write_register(comp_instance->mb_ctx, reg, value);

    if (ret < 0) {
        fprintf(stderr, "t3d_servo: Modbus write error at reg %d: %s", reg, modbus_strerror(errno));

        // If the connection is lost, close it and return failure
        if (modbus_get_socket(comp_instance->mb_ctx) < 0) {
            modbus_close(comp_instance->mb_ctx);
        }
        return ret;
    }
    return ret;
}


int modbus_check_connection() {
    uint16_t test_reg;
    int status = modbus_read_registers(comp_instance->mb_ctx, 76, 1, &test_reg);
    
    if (status == -1) {  // Modbus error
        printf("t3d_servo: Modbus connection lost. Attempting to reconnect...");
        
        modbus_close(comp_instance->mb_ctx);
        modbus_free(comp_instance->mb_ctx);
        
        char *device = find_serial_device();
        if (device == NULL) {
            printf("t3d_servo: No Modbus USB device found.");
            return -1;
        }
        
        comp_instance->mb_ctx = modbus_new_rtu(device, 19200, 'E', 8, 1);
        if (comp_instance->mb_ctx == NULL) {
            printf("t3d_servo: Failed to create Modbus context.");
            return -1;
        }

        modbus_set_slave(comp_instance->mb_ctx, 1);
        if (modbus_connect(comp_instance->mb_ctx) == -1) {
            printf("t3d_servo: Reconnection failed.");
            modbus_free(comp_instance->mb_ctx);
            return -1;
        }

        printf("t3d_servo: Reconnected to Modbus device.");
    }
    
    return 0; // Connection OK
}