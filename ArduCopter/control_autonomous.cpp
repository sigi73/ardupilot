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
#define BARO_AVERAGE_READING_NUM 30
#define TARGET_HEIGHT_LOW        0.6
#define TARGET_HEIGHT_HIGH       1.0
#define ASCENDING_THROTTLE       0.6
#define DESCENDING_THROTTLE      0.2
#define HOVER_THROTTLE           0.4

uint8_t baro_reading_count = 0;
float baro_average;
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

        float output;
        if (baro_average < TARGET_HEIGHT_LOW)
            output = ASCENDING_THROTTLE;
        else if (baro_average > TARGET_HEIGHT_HIGH)
            output = DESCENDING_THROTTLE;
        else
            output = HOVER_THROTTLE;

        attitude_control->set_throttle_out(output, false, g.throttle_filt);
        printf("baro_average: %5.2f, throttle_output: %.2f\n", baro_average, output);
    }
}
