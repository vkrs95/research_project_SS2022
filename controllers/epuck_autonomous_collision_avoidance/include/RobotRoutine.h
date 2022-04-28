#include "epuck_autonomous_collision_avoidance.h"
#pragma once

class RobotRoutine
{
	private:
		std::array<DistanceSensor*, NB_DIST_SENS> proximSensors;
		std::array<DistanceSensor*, NB_GROUND_SENS> groundSensors;
		Camera* robotCamera;

		double LFM_K_GS_SPEED;

	public:
		char name[20];
		int speed[2];

		// name
		std::string robotName;
		std::string qrImgFileName;

		int lfm_speed[2];
		int LFM_FORWARD_SPEED;

		Motor* motorLeft;
		Motor* motorRight;

		std::array<LED*, NB_LEDS> robotLEDs;
		unsigned int activeLED = 0;

		int ps_value[NB_DIST_SENS] = { 0, 0, 0, 0, 0, 0, 0, 0 };
		unsigned short gs_value[NB_GROUND_SENS] = { 0, 0, 0 };

		RobotRoutine(Robot* robot);
		void ReadSensors();
		void LineFollowingModule(void);
		bool DetectLineCrossroad(void);
		bool DetectEndOfLine(void);
		void setWheelSpeedMoveStraightAhead(void);
		void setWheelSpeedTurnLeft(void);
		void setWheelSpeedTurnRight(void);
		void setWheelSpeedTurnAround(void);
		void CyclicBlinkingLED(void);
		void AllLightsOnLED(void);
		void SetSpeedAndVelocity(void);
		void PerformHalt(void);
		void EnableEpuckCam(void);
		void DisableEpuckCam(void);
		void TakeCameraSnapshot(void);
		bool IsEpuckCamEnabled(void);
};

