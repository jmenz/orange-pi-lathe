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
    hal_bit_t *enable;              // Enable the whole component
    hal_float_t *spindle_speed;     // Speed command (RPM)
    hal_bit_t *on;                  // Spindle on/off
    hal_bit_t *hold_motor;          // Hold the motor when not rotating
    hal_bit_t *forward;             // Forward (CCW)
    hal_bit_t *reverse;             // Reverse (CW)
    hal_bit_t *alarm_flag;          // True if alarm_code > 0
    hal_s32_t *alarm_code;          // Read error code
    hal_u32_t *motor_release_delay; // Delay in ms that determines how long to hold motor after stop

    modbus_t *mb_ctx;               // Modbus context
    float last_speed;               // Last written speed (avoid redundant writes)
    int last_command;               // Last recorded control

    int modbus_reconnect_attempts; //number of attempts to reconnect to modbus

    bool modbus_inited;             //is modbus inited and ready to work
    bool last_on_status;

    rtapi_u64 last_modbus_read_time;
} t3d_servo_t;

// 🔹 Define Conponent params
#define MAIN_LOOP_PERIOD 100000  // 100ms (10 Hz polling rate)
#define READ_CYCLE_PERIOD 1000000000 // Every 1 second (1,000,000,000 nanoseconds)

// 🔹 Define Modbus constants
#define MODBUS_READ_REGISTERS_NUM  1   // Number of registers to read at once
#define MODBUS_MAX_RECONNECT_ATTEMPTS 3

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
t3d_servo_t *init_hal_component(int *comp_id);
void handle_sigint(int sig);
int init_hal_pins(t3d_servo_t *comp, int comp_id);
void main_loop(t3d_servo_t *comp);
void servo_write(t3d_servo_t *comp);
void servo_read(t3d_servo_t *comp);

void update_speed(t3d_servo_t *comp);
void update_motor_status(t3d_servo_t *comp);
void send_motor_command(t3d_servo_t *comp, uint16_t command);
void read_alarm(t3d_servo_t *comp);
void check_on_status(t3d_servo_t *comp);


int init_modbus(t3d_servo_t *comp);
int modbus_03_read(t3d_servo_t *comp, int reg, uint16_t *value);
int modbus_04_read(t3d_servo_t *comp, int reg, uint16_t *value);
int modbus_06_write(t3d_servo_t *comp, int reg, uint16_t value);
void handle_modbus_failure(t3d_servo_t *comp);

char *find_serial_device();

#endif // T3D_SERVO_H