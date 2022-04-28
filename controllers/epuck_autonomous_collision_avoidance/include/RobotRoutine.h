#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>

#include <array>
#include <vector>
#include <stdexcept>

#pragma once


// All the webots classes are defined in the "webots" namespace
using namespace webots;

class RobotRoutine
{

public:
	// Global Defines
	const static int NO_SIDE = -1;
	const static unsigned int LEFT = 0;
	const static unsigned int RIGHT = 1;
	const static unsigned int WHITE = 0;
	const static unsigned int BLACK = 1;
	const static unsigned int TIME_STEP = 32;  // [ms]

	// 8 IR proximity sensors
	const static unsigned int NB_DIST_SENS = 8;
	const static unsigned int PS_RIGHT_00 = 0;
	const static unsigned int PS_RIGHT_45 = 1;
	const static unsigned int PS_RIGHT_90 = 2;
	const static unsigned int PS_RIGHT_REAR = 3;
	const static unsigned int PS_LEFT_REAR = 4;
	const static unsigned int PS_LEFT_90 = 5;
	const static unsigned int PS_LEFT_45 = 6;
	const static unsigned int PS_LEFT_00 = 7;

	const int PS_OFFSET_SIMULATION[NB_DIST_SENS] = { 300, 300, 300, 300, 300, 300, 300, 300 };
	const int PS_OFFSET_REALITY[NB_DIST_SENS] = { 480, 170, 320, 500, 600, 680, 210, 640 };

	// 3 IR ground color sensors
	const static unsigned int NB_GROUND_SENS = 3;
	const static unsigned int GS_BLACK = 310;
	const static unsigned int GS_GROUND = 800;
	const static unsigned int GS_LEFT = 0;
	const static unsigned int GS_CENTER = 1;
	const static unsigned int GS_RIGHT = 2;

	// 8 LEDs
	const static unsigned int NB_LEDS = 8;
	const static unsigned int LED_TIME_STEP = 320; // [ms]

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

private:
	std::array<DistanceSensor*, NB_DIST_SENS> proximSensors;
	std::array<DistanceSensor*, NB_GROUND_SENS> groundSensors;
	Camera* robotCamera;

	double LFM_K_GS_SPEED;
};

