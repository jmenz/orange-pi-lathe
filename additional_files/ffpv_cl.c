/**
 * This component implements a hybrid control law:
 * During Motion: Output = (1 + Kv) * vel_cmd + Kp * pos_error
 * Stationary:    If error > deadband, Output = pos_error + dither
 *
 * To compile this component:
 * halcompile --install ffpv_cl.c
 *
 * To load it in your HAL file:
 * loadrt ffpv_cl names=x,z
 *
 * Or by count:
 * loadrt ffpv_cl count=2
 *
 */

#include "rtapi.h"
#include "rtapi_app.h"
#include "hal.h"
#include "rtapi_math.h"

MODULE_AUTHOR("Yevhen Zakharchuk");
MODULE_DESCRIPTION("Feedforward Proportional Velocity Controller with Deadband Positioning");
MODULE_LICENSE("GPL");

#define MAX_INSTANCES 16

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
    hal_float_t deadband;           // PARAMETER: Acceptable stationary position error
    hal_float_t max_correction;     // PARAMETER: Maximum correction that can be applied to velocity
    hal_float_t dither_amp;         // PARAMETER: Amplitude of dither for stationary hold

} ffpv_cl_data;


static int comp_id;
static ffpv_cl_data *data;
static int num_instances;


static void update(void *arg, long period);

static int count = 0;
RTAPI_MP_INT(count, "Number of ffpv_cl instances (used if names is not specified)");
static char *names[MAX_INSTANCES] = {0,};
RTAPI_MP_ARRAY_STRING(names, MAX_INSTANCES, "Names for ffpv_cl instances, comma-separated");


/**
 * @param arg Pointer to the instance data
 * @param period The servo thread period in nanoseconds.
 */
static void update(void *arg, long period) {
    int i;

    for (i = 0; i < num_instances; i++) {
        ffpv_cl_data *inst = &data[i];
        double pos_error, correction_by_pos, correction_by_vel, final_correction;

        // If disabled, output zero
        if (!*(inst->enable)) {
            *(inst->vel_out) = 0.0;
            *(inst->vel_correction) = 0.0;
            continue;
        }

        pos_error = *(inst->pos_cmd) - *(inst->pos_fb);
        
        correction_by_pos = inst->Kp * pos_error;

        if (*(inst->vel_cmd) == 0.0) {

            if (fabs(pos_error) > inst->deadband) {

                long long now = rtapi_get_time();
                double dither_component = ((now % 201) - 100) / 100.0;
                dither_component *= inst->dither_amp;
                double dithered_speed = fabs(correction_by_pos) + dither_component;
                if (dithered_speed < 0) dithered_speed = 0;
                *(inst->vel_out) = (pos_error > 0) ? dithered_speed : -dithered_speed;

            } else {
                *(inst->vel_out) = 0.0;
            }
        } else {

            correction_by_vel = inst->Kv * *(inst->vel_cmd);

            final_correction = correction_by_vel + correction_by_pos;
            if (inst->max_correction > 0 && fabs(final_correction) > inst->max_correction) {
                final_correction = inst->max_correction;
            }

            *(inst->vel_out) = *(inst->vel_cmd) + final_correction;
        }

        // Update the correction output pin for debugging
        *(inst->vel_correction) = *(inst->vel_out) - *(inst->vel_cmd);
    }
}

int rtapi_app_main(void) {
    int i, retval;
    char name_buf[HAL_NAME_LEN + 1];

    if (count && names[0]) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: 'count=' and 'names=' are mutually exclusive.\n");
        return -1;
    }

    if (names[0]) {
        num_instances = 0;
        for (i = 0; i < MAX_INSTANCES; i++) {
            if (names[i] == NULL || *names[i] == 0) {
                break;
            }
            num_instances = i + 1;
        }
    } else {
        num_instances = count;
    }

    if (num_instances <= 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: No instances to create. Use 'count=' or 'names='.\n");
        return -1;
    }

    if (num_instances > MAX_INSTANCES) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: num_instances exceeds MAX_INSTANCES (%d)\n", MAX_INSTANCES);
        return -1;
    }

    comp_id = hal_init("ffpv_cl");
    if (comp_id < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: hal_init() failed\n");
        return -1;
    }

    data = hal_malloc(num_instances * sizeof(ffpv_cl_data));
    if (data == 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv_cl: ERROR: hal_malloc() failed for data\n");
        hal_exit(comp_id);
        return -1;
    }

    for (i = 0; i < num_instances; i++) {
        char *instance_name;
        if (names[0]) {
            instance_name = names[i];
        } else {
            rtapi_snprintf(name_buf, sizeof(name_buf), "%d", i);
            instance_name = name_buf;
        }

        // --- Create INPUT pins ---
        retval = hal_pin_float_newf(HAL_IN, &(data[i].pos_cmd), comp_id, "ffpv-cl.%s.pos-cmd", instance_name);
        if(retval < 0) goto error;
        retval = hal_pin_float_newf(HAL_IN, &(data[i].pos_fb), comp_id, "ffpv-cl.%s.pos-fb", instance_name);
        if(retval < 0) goto error;
        retval = hal_pin_float_newf(HAL_IN, &(data[i].vel_cmd), comp_id, "ffpv-cl.%s.vel-cmd", instance_name);
        if(retval < 0) goto error;
        retval = hal_pin_bit_newf(HAL_IN, &(data[i].enable), comp_id, "ffpv-cl.%s.enable", instance_name);
        if(retval < 0) goto error;

        // --- Create OUTPUT pins ---
        retval = hal_pin_float_newf(HAL_OUT, &(data[i].vel_out), comp_id, "ffpv-cl.%s.vel-out", instance_name);
        if(retval < 0) goto error;
        retval = hal_pin_float_newf(HAL_OUT, &(data[i].vel_correction), comp_id, "ffpv-cl.%s.vel-correction", instance_name);
        if(retval < 0) goto error;

        // --- Create PARAMETERS ---
        retval = hal_param_float_newf(HAL_RW, &(data[i].Kp), comp_id, "ffpv-cl.%s.Kp", instance_name);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].Kv), comp_id, "ffpv-cl.%s.Kv", instance_name);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].deadband), comp_id, "ffpv-cl.%s.deadband", instance_name);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].max_correction), comp_id, "ffpv-cl.%s.max-correction", instance_name);
        if(retval < 0) goto error;
        retval = hal_param_float_newf(HAL_RW, &(data[i].dither_amp), comp_id, "ffpv-cl.%s.dither-amp", instance_name);
        if(retval < 0) goto error;


        // --- Set default parameter values ---
        data[i].Kp = 1.0;
        data[i].Kv = 0.0;
        data[i].deadband = 0.001; // Default to 1 micron, should be tuned
        data[i].max_correction = 0.0;
        data[i].dither_amp = 0.0;
        *(data[i].enable) = 1;
    }
    
    rtapi_snprintf(name_buf, sizeof(name_buf), "ffpv-cl.update");
    retval = hal_export_funct(name_buf, update, data, 1, 0, comp_id);
    if (retval < 0) {
        rtapi_print_msg(RTAPI_MSG_ERR, "ffpv-cl: ERROR: hal_export_funct() failed\n");
        goto error;
    }

    rtapi_print_msg(RTAPI_MSG_INFO, "ffpv-cl: INFO: Loaded %d instances\n", num_instances);
    hal_ready(comp_id);
    return 0;

error:
    hal_exit(comp_id);
    return -1;
}

void rtapi_app_exit(void) {
    hal_exit(comp_id);
}
