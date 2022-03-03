#include "ObstacleAvoidance.h"

bool ObstacleAvoidance::ObstacleDetection(int ps_value[NB_DIST_SENS])
{
    int max_ds_value, DeltaS = 0, i;
    int Activation[] = { 0,0 };

    if (oam_reset)
    {
        oam_active = false;
        oam_side = NO_SIDE;
    }

    oam_reset = 0;
    max_ds_value = 0;

    for (i = PS_RIGHT_00; i <= PS_RIGHT_45; i++) 
    {
        if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
        Activation[RIGHT] += ps_value[i];
    }

    for (i = PS_LEFT_45; i <= PS_LEFT_00; i++) 
    {
        if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
        Activation[LEFT] += ps_value[i];
    }

    if (max_ds_value > OAM_OBST_THRESHOLD) 
    {
        return true;
    }

    return false;
}
