#ifndef T3D_SERVO_H
#define T3D_SERVO_H

#include <rtapi.h>
#include <rtapi_app.h>
#include <hal.h>
#include <glob.h>
#include <string.h>
#include <modbus/modbus.h>

// Define HAL Component Structure
typedef struct {
    hal_float_t *spindle_speed;   // Speed command (RPM)
    hal_bit_t *on;              // Spindle on/off
    hal_bit_t *forward;         // Forward (CCW)
    hal_bit_t *reverse;         // Reverse (CW)
    hal_bit_t *error_flag;      // True if error_code > 0
    hal_s32_t *error_code;      // Read error code

    modbus_t *mb_ctx;           // Modbus context
    float last_speed;             // Last written speed (avoid redundant writes)
    int last_control;           // Last recorded control
    char device;                // device name (/dev/ttyUSB0)
    int slave_num;              // index number of device
} t3d_servo_t;

// Function Prototypes
char *find_serial_device(); 
int init(void);
void update(void *arg, long period);

#endif // T3D_SERVO_H