#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>

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

	// name
	std::string robotName;
	std::string qrImgFileName;

	// robot time step
	int basicTimeStep = 0;

	/* proximity sensor values, accessed by obstacle avoidance */
	int ps_value[NB_DIST_SENS] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	/* public functions provided by robot routine module */
	RobotRoutine(Robot* robot);
	void ReadSensors();

	/* ground sensor functions */
	void LineFollowingModule(void);
	bool DetectLineCrossroad(void);
	bool DetectEndOfLine(void);
	
	/* motor control functions */
	void setWheelSpeedMoveStraightAhead(void);
	void setWheelSpeedTurnLeft(void);
	void setWheelSpeedTurnRight(void);
	void setWheelSpeedTurnAround(void);
	void SetSpeedAndVelocity(void);
	void PerformHalt(void);

	/* LED functions */
	void CyclicBlinkingLED(void);
	void AllLightsOnLED(void);
	
	/* camera functions */
	void EnableEpuckCam(void);
	void DisableEpuckCam(void);
	void TakeCameraSnapshot(void);
	bool IsEpuckCamEnabled(void);

	/* obstacle functions */
	bool detectObstacle(void);

	/* receiver functions */
	bool getNextPacket(int* dataPacket);

private:
	/* robot sensor objects utilized by functions */
	std::array<DistanceSensor*, NB_DIST_SENS> proximSensors;
	std::array<DistanceSensor*, NB_GROUND_SENS> groundSensors;

	Motor* motorLeft;
	Motor* motorRight;

	Camera* robotCamera;

	Receiver* receiver;

	unsigned short gs_value[NB_GROUND_SENS] = { 0, 0, 0 };

	int lfm_speed[2];

	unsigned int activeLED = 0;
	std::array<LED*, NB_LEDS> robotLEDs;

	/* internal constants */
	const double LFM_K_GS_SPEED = 0.4;
	const int LFM_FORWARD_SPEED = 200;
	const double MOTOR_RATIO = 0.00628;

	/* internal constants obstacle avoidance module */
	const static unsigned int OAM_OBST_THRESHOLD = 100;
	const static unsigned int OAM_FORWARD_SPEED = 150;
	const static unsigned int OAM_K_MAX_DELTAS = 600;
	const double OAM_K_PS_90 = 0.2;
	const double OAM_K_PS_45 = 0.9;
	const double OAM_K_PS_00 = 1.2;

	/* internal constant receiver */
	const static unsigned int RECV_SAMPLING_PERIOD = 64; 
};

