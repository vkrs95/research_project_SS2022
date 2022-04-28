#include "ObstacleAvoidance.h"

bool ObstacleAvoidance::ObstacleDetection(int ps_value[RobotRoutine::NB_DIST_SENS])
{
    int max_ds_value, DeltaS = 0, i;
    int Activation[] = { 0,0 };

    if (oam_reset)
    {
        oam_active = false;
        // oam_side = NO_SIDE; // set for what purpose?
    }

    oam_reset = 0;
    max_ds_value = 0;

    for (i = RobotRoutine::PS_RIGHT_00; i <= RobotRoutine::PS_RIGHT_45; i++)
    {
        if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
        Activation[RobotRoutine::RIGHT] += ps_value[i];
    }

    for (i = RobotRoutine::PS_LEFT_45; i <= RobotRoutine::PS_LEFT_00; i++)
    {
        if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
        Activation[RobotRoutine::LEFT] += ps_value[i];
    }

    if (max_ds_value > OAM_OBST_THRESHOLD) 
    {
        return true;
    }

    return false;
}
