#include "t3d_servo.h"

t3d_servo_t *comp_instance;  // Component instance
static int comp_id; // Store the component ID


int main(int argc, char *argv[]) {
    // Register signal handler for clean exit
    signal(SIGINT, handle_sigint);
    signal(SIGTERM, handle_sigint);

    // Initialize HAL component
    comp_id = hal_init("t3d_servo");
    if (comp_id < 0) {
        fprintf(stderr, "t3d_servo: Initialization failed\n");
        return 1;
    }

    if (init() < 0) {
        fprintf(stderr, "t3d_servo: Initialization failed\n");
        return 1;
    }

    // Start a background thread for Modbus communication
    pthread_t thread;
    pthread_create(&thread, NULL, modbus_thread, comp_instance);
    pthread_detach(thread);  // Don't wait for it to finish

    // Finalize HAL component setup
    hal_ready(comp_id);

    printf("t3d_servo: Loaded successfully");

    // Keep the process running indefinitely (like other LinuxCNC user-space components)
    while (1) {
        sleep(1);
    }

    return 0;
}

int init(void) {
    int retval;

    // Allocate HAL memory for component
    comp_instance = hal_malloc(sizeof(t3d_servo_t));
    if (!comp_instance) {
        printf("t3d_servo: HAL memory allocation failed");
        return -1;
    }

    retval = init_hal_pins(&comp_id);
    if (retval < 0) {
        printf("t3d_servo: hal pin creation failed");
        return retval;
    } 

    comp_instance->last_speed = 0;
    comp_instance->last_control = 0;

    char *device = find_serial_device();
    if (device == NULL) {
        printf("t3d_servo: No Modbus USB device found!");
        return -1;
    }

    fprintf(stderr, "t3d_servo: Using Modbus device %s", device);

    // Initialize Modbus connection
    comp_instance->mb_ctx = modbus_new_rtu(
        device,
        t3d_modbus_params.baud,
        t3d_modbus_params.parity,
        t3d_modbus_params.data_bit,
        t3d_modbus_params.stop_bit
    );

    if (!comp_instance->mb_ctx) {
        printf("t3d_servo: Failed to create Modbus context");
        return -1;
    }

    modbus_set_slave(comp_instance->mb_ctx, t3d_modbus_params.slave);  // Use configured slave number
    if (modbus_connect(comp_instance->mb_ctx) == -1) {
        printf("t3d_servo: Failed to connect to Modbus device");
        return -1;
    }

    return 0;
}


void *modbus_thread(void *arg) {
    while (1) {
        update(arg);  // Call the update function
        usleep(100000);  // Sleep for 100ms (10 Hz polling rate)
    }
    return NULL;
}

void update(void *arg) {
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

void update_speed(t3d_servo_t *comp) {
    // Only send speed if it changed (MODBUS_REG_RPM, Function 06)
    if (*(comp->spindle_speed) != comp->last_speed) {
        uint16_t speed_val = *(comp->spindle_speed);

        if (modbus_06_write(MODBUS_REG_RPM, speed_val) >= 0) {
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
        if (modbus_06_write(MODBUS_REG_CONTROL, control_val) >= 0) {
            comp->last_control = control_val;
        }
    }
}

void read_alarm(t3d_servo_t *comp) {
    
    uint16_t alarm_code;

    if (modbus_04_read(MODBUS_REG_ALARM, &alarm_code) >= 0) {
        *comp->alarm_code = alarm_code;
        *comp->alarm_flag = (alarm_code > 0) ? 1 : 0;
    }
}


// Signal handler to clean up before exit
void handle_sigint(int sig) {
    printf("t3d_servo: Exiting...\n");
    if (comp_instance && comp_instance->mb_ctx) {
        modbus_06_write(MODBUS_REG_CONTROL, t3d_servo_control.off);
        modbus_close(comp_instance->mb_ctx);
        modbus_free(comp_instance->mb_ctx);
    }
    hal_exit(comp_id);
    exit(0);
}