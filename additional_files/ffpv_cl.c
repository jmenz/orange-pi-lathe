/**
 * This component implements a hybrid control law:
 * During Motion: Output = (1 + Kv) * vel_cmd + Kp * pos_error
 * Stationary:    If error > deadband, Output = final_min_speed
 *
 * To compile this component:
 * halcompile --install ffpv_cl.c
 *
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "rtapi_math.h"

MODULE_AUTHOR("Yevhen Zakharchuk");
MODULE_DESCRIPTION("Feedforward Proportional Velocity Controller with Deadband Positioning");
MODULE_LICENSE("GPL");

// --- Component Data Structure ---
typedef struct {
    // HAL Pin Pointers
    hal_float_t *pos_cmd;        // INPUT: Commanded position from motion planner
    hal_float_t *pos_fb;         // INPUT: Feedback position from encoder
    hal_float_t *vel_cmd;        // INPUT: Commanded velocity (feedforward term)
    hal_bit_t   *enable;         // INPUT: Enable bit for the component

    hal_float_t *vel_out;        // OUTPUT: The final velocity command for the motor
    hal_float_t *vel_correction; // OUTPUT: velocity correction value

    // HAL Parameter Values
    hal_float_t Kp;                 // PARAMETER: Proportional gain on position error
    hal_float_t Kv;                 // PARAMETER: Gain for the velocity feedforward term
    hal_float_t deadband;     // PARAMETER: Acceptable stationary position error
    hal_float_t min_speed;    // PARAMETER: Minimum speed to overcome stepgen deadband
    hal_float_t dither_amp;   // PARAMETER: Amplitude of dither for stationary hold

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
        double pos_error, correction_by_pos, correction_by_vel;

        // If disabled, output zero
        if (!*(inst->enable)) {
            *(inst->vel_out) = 0.0;
            *(inst->vel_correction) = 0.0;
            continue;
        }

        // 1. Calculate the position error
        pos_error = *(inst->pos_cmd) - *(inst->pos_fb);

        // 2. Handle Stationary vs. Motion logic
        if (*(inst->vel_cmd) == 0.0) {
            // --- STATIONARY HOLD (Deadband logic) ---

            // Check if the absolute error is outside the acceptable deadband
            if (fabs(pos_error) > inst->deadband) {

                long long now = rtapi_get_time();
                // Create a pseudo-random float between -1.0 and 1.0
                double dither_component = ((now % 201) - 100) / 100.0;
                // Scale by the dither amplitude parameter
                dither_component *= inst->dither_amp;

                // Add the dither to the minimum speed
                double dithered_speed = inst->min_speed + dither_component;
                
                // Ensure dithered speed is not negative
                if (dithered_speed < 0) dithered_speed = 0;

                // Command the move with the dithered speed
                *(inst->vel_out) = (pos_error > 0) ? dithered_speed : -dithered_speed;

            } else {
                // Error is within the deadband, command a stop.
                *(inst->vel_out) = 0.0;
            }
        } else {
            // --- IN MOTION (FF+P active) ---

            // Calculate the correction velocity based on the position error and Kp
            correction_by_pos = inst->Kp * pos_error;

            // Calculate the correction velocity based on the Kv gain
            correction_by_vel = inst->Kv * *(inst->vel_cmd);

            // Sum all terms to get the final output
            *(inst->vel_out) = *(inst->vel_cmd) + correction_by_vel + correction_by_pos;
        }

        // 3. Update the correction output pin for debugging
        *(inst->vel_correction) = *(inst->vel_out) - *(inst->vel_cmd);
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
        retval = hal_pin_float_newf(HAL_OUT, &(data[i].vel_correction), comp_id, "ffpv-cl.%d.vel-correction", i);
        if(retval < 0) goto error;

        // --- Create PARAMETERS ---
        retval = hal_param_float_newf(HAL_RW, &(data[i].Kp), comp_id, "ffpv-cl.%d.Kp", i);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].Kv), comp_id, "ffpv-cl.%d.Kv", i);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].deadband), comp_id, "ffpv-cl.%d.deadband", i);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].min_speed), comp_id, "ffpv-cl.%d.min-speed", i);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].dither_amp), comp_id, "ffpv-cl.%d.dither-amp", i);
        if(retval < 0) goto error;


        // --- Set default parameter values ---
        data[i].Kp = 1.0;
        data[i].Kv = 0.0;
        data[i].deadband = 0.001; // Default to 1 micron, user should tune
        data[i].min_speed = 0.01; // Default to a slow speed, user must tune
        data[i].dither_amp = 0.0;
        *(data[i].enable) = 1;
    }

    rtapi_snprintf(name, sizeof(name), "ffpv-cl.update");
    retval = hal_export_funct(name, update, data, 1, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv-cl: ERROR: hal_export_funct() failed\n");
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
