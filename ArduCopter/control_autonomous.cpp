#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::autonomous_init(bool ignore_checks)
{
    // if landed and the mode we're switching from does not have manual throttle and the throttle stick is too high
    if (motors->armed() && ap.land_complete && !mode_has_manual_throttle(control_mode) &&
            (get_pilot_desired_throttle(channel_throttle->get_control_in()) > get_non_takeoff_throttle())) {
        return false;
    }
    // set target altitude to zero for reporting
    pos_control->set_alt_target(0);

    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
#if 0
#define BARO_AVERAGE_READING_NUM 1
#define TARGET_HEIGHT_LOW        0.6
#define TARGET_HEIGHT_HIGH       1.0
#define TARGET_HEIGHT            1.0
#define ASCENDING_THROTTLE       0.6
#define DESCENDING_THROTTLE      0.2
#define HOVER_THROTTLE           0.4

uint8_t baro_reading_count = 0;
float baro_average;
#endif

//float kp = 0.1f;
//float ki = 0.0f;
//float kd = 0.0f;


void Copter::autonomous_run()
{
    float outMax = g2.alt_outMax;
    float outMin = g2.alt_outMin;

    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    float height = (float)rangefinder_state.alt_cm / 100.0f;
    float error = g2.alt_target_height - height;
    static float last_error;
    static float error_sum;
    error_sum += error;
    float iTerm = g2.alt_i * error_sum * G_Dt;

    if (iTerm > outMax) iTerm = outMax;
    if (iTerm < outMin) iTerm = outMin;
    float output = g2.alt_p * error + iTerm + g2.alt_d * last_error / G_Dt;
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    last_error = error;

    output += 0.5;

    attitude_control->set_throttle_out(output, false, g.throttle_filt);
    printf("rangefinder: %5.2f, throttle_output: %.2f\n", height, output);
    //printf("p: %f, i: %f, d: %f\n", (float)g2.alt_p, (float)g2.alt_i, (float)g2.alt_d);
}

#if 0
void Copter::autonomous_run()
{
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // convert pilot input to lean angles
    // To-Do: convert get_pilot_desired_lean_angles to return angles as floats
    //get_pilot_desired_lean_angles(channel_roll->get_control_in(), channel_pitch->get_control_in(), target_roll, target_pitch, aparm.angle_max);

    // get pilot's desired yaw rate
    //target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());

    // get pilot's desired throttle
    //pilot_throttle_scaled = get_pilot_desired_throttle(channel_throttle->get_control_in());

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0, get_smoothing_gain());

    // body-frame rate controller is run directly from 100hz loop

    // output pilot's throttle
    //attitude_control->set_throttle_out(pilot_throttle_scaled, true, g.throttle_filt);
    //Average over several frames
    if (baro_reading_count < BARO_AVERAGE_READING_NUM)
    {
        baro_reading_count++;
        baro_average += barometer.get_altitude();
    }
    else
    {
        baro_reading_count = 0;
        baro_average /= BARO_AVERAGE_READING_NUM;


        float error = TARGET_HEIGHT - baro_average;
        static float last_error;
        static float error_sum;
        error_sum += error;
        float iTerm = g2.alt_i * error_sum * G_Dt;

        if (iTerm > outMax) iTerm = outMax;
        if (iTerm < outMin) iTerm = outMin;
        float output = g2.alt_p * error + iTerm + g2.alt_d * last_error / G_Dt;
        if (output > outMax) output = outMax;
        if (output < outMin) output = outMin;

        last_error = error;

        output += 0.5;

        /*
        if (baro_average < TARGET_HEIGHT_LOW)
            output = ASCENDING_THROTTLE;
        else if (baro_average > TARGET_HEIGHT_HIGH)
            output = DESCENDING_THROTTLE;
        else
            output = HOVER_THROTTLE;
        */

        attitude_control->set_throttle_out(output, false, g.throttle_filt);
        printf("baro_average: %5.2f, throttle_output: %.2f\n", baro_average, output);
        baro_average = 0.0f;
        //printf("p: %f, i: %f, d: %f\n", (float)g2.alt_p, (float)g2.alt_i, (float)g2.alt_d);
    }
}
#endif
