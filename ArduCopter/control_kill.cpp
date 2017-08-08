#include "Copter.h"

/*
 * Init and run calls for stabilize flight mode
 */

// stabilize_init - initialise stabilize controller
bool Copter::kill_init(bool ignore_checks)
{
    return true;
}

// stabilize_run - runs the main stabilize controller
// should be called at 100hz or more
void Copter::kill_run()
{
    init_disarm_motors();
}
