/**
 * This component implements the control law:
 * Output = (Kp * position_error) + (Kv * velocity_command)
 *
 * It is designed to replace a standard PID component for position control loops
 * where direct velocity feedforward is desired.
 *
 * To compile this component:
 * halcompile --install ffpv_cl.c
 *
 * To load it in your HAL file:
 * loadrt ffpv_cl
 *
 * Or to load multiple instances:
 * loadrt ffpv_cl count=3
 *
 */


#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "rtapi_math.h"


MODULE_AUTHOR("Yevhen Zakharchuk");
MODULE_DESCRIPTION("Feedforward Proportional Velocity Closed-Loop Controller");
MODULE_LICENSE("GPL");

// --- Component Data Structure ---
typedef struct {
    // HAL Pin Pointers
    hal_float_t *pos_cmd;       // INPUT: Commanded position from motion planner
    hal_float_t *pos_fb;        // INPUT: Feedback position from encoder
    hal_float_t *vel_cmd;       // INPUT: Commanded velocity (feedforward term)
    hal_bit_t   *enable;        // INPUT: Enable bit for the component

    hal_float_t *vel_out;       // OUTPUT: The final velocity command for the motor
    hal_float_t *error;         // OUTPUT: The calculated position error for scoping

    // HAL Parameter Values
    hal_float_t Kp;            // PARAMETER: Proportional gain on position error
    hal_float_t Kv;            // PARAMETER: Gain for the velocity feedforward term
} ffpv_cl_data;


static int comp_id;
static ffpv_cl_data *data;
static int num_instances = 0;


static void update(void *arg, long period);

static int count = 1;
RTAPI_MP_INT(count, "Number of ffpv_cl instances");

/**
 * @param arg Pointer to the instance data
 * @param period The servo thread period in nanoseconds.
 */
static void update(void *arg, long period) {
    int i;
    for (i = 0; i < num_instances; i++) {
        ffpv_cl_data *inst = &data[i];
        double pos_error, correction_vel, feedforward_vel;

        if (!*(inst->enable)) {
            *(inst->vel_out) = 0.0;
            *(inst->error) = 0.0;
            continue;
        }

        // 1. Calculate the position error
        pos_error = *(inst->pos_cmd) - *(inst->pos_fb);

        // 2. Calculate the correction velocity based on the position error and Kp
        correction_vel = inst->Kp * pos_error;

        // 3. Get the feedforward velocity and apply its gain Kv
        feedforward_vel = inst->Kv * *(inst->vel_cmd);

        // 4. Sum the feedforward and correction velocities to get the final output
        *(inst->vel_out) = feedforward_vel + correction_vel;

        // 5. Update the error output pin for debugging and scoping
        *(inst->error) = pos_error;
    }
}

int rtapi_app_main(void) {
    int i, retval;
    char name[HAL_NAME_LEN + 1];

    if (count < 1) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: count must be > 0\n");
        return -1;
    }
    num_instances = count;

    comp_id = hal_init("ffpv_cl");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: hal_init() failed\n");
        return -1;
    }

    data = hal_malloc(num_instances * sizeof(ffpv_cl_data));
    if (data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: hal_malloc() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    for (i = 0; i < num_instances; i++) {
        // --- Create INPUT pins ---
        retval = hal_pin_float_newf(HAL_IN, &(data[i].pos_cmd), comp_id, "ffpv-cl.%d.pos-cmd", i);
        if(retval < 0) goto error;
        retval = hal_pin_float_newf(HAL_IN, &(data[i].pos_fb), comp_id, "ffpv-cl.%d.pos-fb", i);
        if(retval < 0) goto error;
        retval = hal_pin_float_newf(HAL_IN, &(data[i].vel_cmd), comp_id, "ffpv-cl.%d.vel-cmd", i);
        if(retval < 0) goto error;
        retval = hal_pin_bit_newf(HAL_IN, &(data[i].enable), comp_id, "ffpv-cl.%d.enable", i);
        if(retval < 0) goto error;

        // --- Create OUTPUT pins ---
        retval = hal_pin_float_newf(HAL_OUT, &(data[i].vel_out), comp_id, "ffpv-cl.%d.vel-out", i);
        if(retval < 0) goto error;
        retval = hal_pin_float_newf(HAL_OUT, &(data[i].error), comp_id, "ffpv-cl.%d.error", i);
        if(retval < 0) goto error;

        // --- Create PARAMETERS ---
        retval = hal_param_float_newf(HAL_RW, &(data[i].Kp), comp_id, "ffpv-cl.%d.Kp", i);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].Kv), comp_id, "ffpv-cl.%d.Kv", i);
        if(retval < 0) goto error;

        // --- Set default parameter values ---
        data[i].Kp = 0.1;
        data[i].Kv = 1.0; // Kv should ideally be 1 if scaling is correct
        *(data[i].enable) = 0;
    }

    rtapi_snprintf(name, sizeof(name), "ffpv-cl.update");
    retval = hal_export_funct(name, update, data, 1, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: hal_export_funct() failed\n");
        hal_exit(comp_id);
        return -1;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "ffpv_cl: INFO: Loaded %d instances\n", num_instances);
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return -1;
}

void rtapi_app_exit(void) {
    hal_exit(comp_id);
}