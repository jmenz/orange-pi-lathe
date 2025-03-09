#ifndef T3D_SERVO_H
#define T3D_SERVO_H

#include <rtapi.h>
#include <hal.h>
#include <glob.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>   // For sleep
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <modbus/modbus.h>

// Define HAL Component Structure
typedef struct {
    hal_float_t *spindle_speed;   // Speed command (RPM)
    hal_bit_t *on;                // Spindle on/off
    hal_bit_t *hold_motor;        // Hold the motor when not rotating
    hal_bit_t *forward;           // Forward (CCW)
    hal_bit_t *reverse;           // Reverse (CW)
    hal_bit_t *alarm_flag;        // True if alarm_code > 0
    hal_s32_t *alarm_code;        // Read error code

    modbus_t *mb_ctx;             // Modbus context
    float last_speed;             // Last written speed (avoid redundant writes)
    int last_control;             // Last recorded control
    char device;                  // device name (/dev/ttyUSB0)
    int slave_num;                // index number of device

    rtapi_u64 last_modbus_read_time;
} t3d_servo_t;


// 🔹 Define Register Addresses
#define MODBUS_REG_RPM      76      // RPM Setpoint Register (0x004C)
#define MODBUS_REG_CONTROL  4112    // Control Command Register (0x1010)
#define MODBUS_REG_ALARM    26      // Error Code Register (0x001A)

typedef struct {
    char *device;
    int slave;
    int baud;
    char parity;
    int data_bit;
    int stop_bit;
} t3d_modbus_param_t;

static const t3d_modbus_param_t t3d_modbus_params = {
    .device    = "/dev/ttyUSB0",
    .slave     = 1,
    .baud      = 19200,
    .parity    = 'E',
    .data_bit  = 8,
    .stop_bit  = 1
};

#define MODBUS_READ_REGISTERS_NUM  1   // Number of registers to read at once

// 🔹 Define Control Command Values (from HAL MUX configuration)
typedef struct {
    uint16_t stop;
    uint16_t forward;
    uint16_t reverse;
    uint16_t  off;
} t3d_servo_control_t;

static const t3d_servo_control_t t3d_servo_control = {
    .stop    = 4660,  // STOP Command
    .forward = 8738,  // FORWARD Command
    .reverse = 4369,  // REVERSE Command
    .off     = 0
};


// Function Prototypes
void handle_sigint(int sig);
int init_hal_component();
int init_hal_pins(t3d_servo_t *comp);
void main_loop(t3d_servo_t *comp);
void servo_write(t3d_servo_t *comp);
void servo_read(t3d_servo_t *comp);

void update_speed(t3d_servo_t *comp);
void update_control(t3d_servo_t *comp);
void read_alarm(t3d_servo_t *comp);


int init_modbus(t3d_servo_t *comp);
int modbus_03_read(t3d_servo_t *comp, int reg, uint16_t *value);
int modbus_04_read(t3d_servo_t *comp, int reg, uint16_t *value);
int modbus_06_write(t3d_servo_t *comp, int reg, uint16_t value);

char *find_serial_device();
int modbus_check_connection(t3d_servo_t *comp);

#endif // T3D_SERVO_H