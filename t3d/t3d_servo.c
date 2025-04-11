#include "t3d_servo.h"

t3d_servo_t *comp_instance;  // Component instance
static int comp_id; // Store the component ID


int main(int argc, char *argv[]) {
    // Register signal handler for clean exit
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    readParams(argc, argv);

    // Initialize HAL Component & Get Instance
    comp_instance = init_hal_component(&comp_id);
    if (!comp_instance) {
        fprintf(stderr, "T3D_SERVO: HAL component initialization failed. Exiting...\n");
        return 1;
    }

    // Finalize HAL component setup
    hal_ready(comp_id);
    fprintf(stdout, "T3D_SERVO: Loaded successfully\n");

    main_loop(comp_instance);

    return 0;
}

void main_loop(t3d_servo_t *comp) {

    while (1) {
        if (!*(comp->enable)) {
            // turn off if was inited
            if (comp->modbus_inited) {
                modbus_06_write(comp_instance, MODBUS_REG_CONTROL, t3d_servo_control.off);
                comp->modbus_inited = false;
                comp->modbus_reconnect_attempts = 0;
            }

            usleep(MAIN_LOOP_PERIOD);
            continue;
        }

        watch_reset_alert_signal(comp);
        check_on_status(comp);
            
        if (init_modbus(comp_instance) < 0) {
            usleep(MAIN_LOOP_PERIOD);
            continue;
        }

        servo_write(comp);

        // Read Modbus every 1 second (1,000,000,000 nanoseconds)
        rtapi_u64 current_time = rtapi_get_time();
        if ((current_time - comp->last_modbus_read_time) >= READ_CYCLE_PERIOD) {
            servo_read(comp);
            comp->last_modbus_read_time = current_time;
        }

        usleep(MAIN_LOOP_PERIOD);  // Sleep for 100ms (10 Hz polling rate)
    }
}

void check_on_status(t3d_servo_t *comp) {
    if (comp->last_on_status != *(comp->on)) {
        comp->last_on_status = *(comp->on);

        if (*(comp->on) == 1) {
            comp->modbus_reconnect_attempts = 0; //reset reconnect attempts when component turned back on
        }
    }
}

void servo_write(t3d_servo_t *comp) {
    update_motor_status(comp_instance);
    update_speed(comp_instance);
}

void update_speed(t3d_servo_t *comp) {
    // Only send speed if it changed (MODBUS_REG_RPM, Function 06)
    if (*(comp->spindle_speed) != comp->last_speed) {
        uint16_t speed_val = *(comp->spindle_speed);

        if (modbus_06_write(comp, MODBUS_REG_RPM, speed_val) >= 0) {
            comp->last_speed = *(comp->spindle_speed);
        }
    }
}

void update_motor_status(t3d_servo_t *comp) {
    
    if (*(comp->on)) {
        // Determine control command

        uint16_t command = t3d_servo_control.off;
        if (*(comp->hold_motor) == 1) {
            command = t3d_servo_control.stop;
        }

        if (*(comp->forward) && !*(comp->reverse)) {
            command = t3d_servo_control.forward;  // FORWARD
        } else if (*(comp->reverse) && !*(comp->forward)) {
            command = t3d_servo_control.reverse;  // REVERSE
        }
        
        send_motor_command(comp, command);

    } else {
        send_motor_command(comp, t3d_servo_control.off);
    }
}

void send_motor_command(t3d_servo_t *comp, uint16_t command) {
    // Only send control command if it changed
    if (command != comp->last_command) {
        if (modbus_06_write(comp, MODBUS_REG_CONTROL, command) >= 0) {
            comp->last_command = command;
        }
    }
}

void watch_reset_alert_signal(t3d_servo_t *comp) {
    if (*(comp->reset_alarm)) {
        fprintf(stderr,  "T3D_SERVO: reset alarm");
        modbus_06_write(comp, MODBUS_REG_RESET_ALARM, MODBUS_RESET_ALARM_VALUE);
        usleep(RESET_ALARM_DELAY);
    }
}


int update_servo_settings(t3d_servo_t *comp) {
    modbus_06_write(comp, MODBUS_REG_MAX_RPM, comp->speed_limit);
    modbus_06_write(comp, MODBUS_REG_ACCEL_TIME, comp->acceleration_time);
    modbus_06_write(comp, MODBUS_REG_DECEL_TIME, comp->deceleration_time);
}


// -------------------- Read Section ---------------------
void servo_read(t3d_servo_t *comp) {
    read_alarm(comp_instance);
}

void read_alarm(t3d_servo_t *comp) {
    uint16_t alarm_code;

    if (modbus_04_read(comp, MODBUS_REG_ALARM, &alarm_code) >= 0) {
        if (*comp->alarm_code != alarm_code) {
            *comp->alarm_code = alarm_code;
            *comp->alarm_flag = (alarm_code > 0) ? 1 : 0;
        }
    }
}

//--------------------General functions

// Signal handler to clean up before exit
void handle_sigint(int sig) {
    if (comp_instance && comp_instance->mb_ctx) {
        modbus_06_write(comp_instance, MODBUS_REG_CONTROL, t3d_servo_control.off);
        modbus_close(comp_instance->mb_ctx);
        modbus_free(comp_instance->mb_ctx);
    }
    hal_exit(comp_id);
    exit(0);
}