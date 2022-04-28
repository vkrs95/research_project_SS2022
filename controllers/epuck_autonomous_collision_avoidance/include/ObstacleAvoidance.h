#include "RobotRoutine.h"
#pragma once

constexpr auto OAM_OBST_THRESHOLD = 100;
constexpr auto OAM_FORWARD_SPEED = 150;
constexpr auto OAM_K_PS_90 = 0.2;
constexpr auto OAM_K_PS_45 = 0.9;
constexpr auto OAM_K_PS_00 = 1.2;
constexpr auto OAM_K_MAX_DELTAS = 600;

class ObstacleAvoidance
{
	private:
		int oam_active, oam_reset;
		int oam_speed[2];
		int oam_side = RobotRoutine::NO_SIDE;

	public:
		bool ObstacleDetection(int ps_value[RobotRoutine::NB_DIST_SENS]);
		// ToDo: all the avoidance routines
};

