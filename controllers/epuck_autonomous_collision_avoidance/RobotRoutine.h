#include "epuck_autonomous_collision_avoidance.h"
#pragma once

class RobotRoutine
{
	private:
		std::array<DistanceSensor*, NB_DIST_SENS> proxim_sensors;
		std::array<DistanceSensor*, NB_GROUND_SENS> ground_sensors;
		Camera* epuck_cam;

		double LFM_K_GS_SPEED;

	public:
		char name[20];
		int speed[2];

		// name
		std::string epuck_name;
		std::string qr_img_file_name;

		int lfm_speed[2];
		int LFM_FORWARD_SPEED;

		Motor* motor_left;
		Motor* motor_right;

		std::array<LED*, NB_LEDS> robot_leds;
		unsigned int active_led = 0;

		int ps_value[NB_DIST_SENS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
		unsigned short gs_value[NB_GROUND_SENS] = { 0, 0, 0 };

		RobotRoutine(Robot* robot);
		void ReadSensors();
		void LineFollowingModule(void);
		bool DetectLineCrossroad(void);
		bool DetectEndOfLine(void);
		void OnCrossroadTurnLeft(void);
		void OnCrossroadTurnRight(void);
		void OnCrossroadTurnDegree(void);
		void CyclicBlinkingLED(void);
		void AllLightsOnLED(void);
		void SetSpeedAndVelocity(void);
		void PerformTurnAround(void);
		void PerformHalt(void);
		void EnableEpuckCam(void);
		void DisableEpuckCam(void);
		void TakeCameraSnapshot(void);
};

