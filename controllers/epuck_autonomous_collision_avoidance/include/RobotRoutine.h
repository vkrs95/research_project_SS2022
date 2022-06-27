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

	std::string qrImgFileName;

	// robot time step
	int basicTimeStep = 0;


	/*********************************************************
	*
	*	public functions provided by robot routine module
	*
	*********************************************************/
	
	RobotRoutine(Robot* robot);
	
	/* general functions */
	std::string getRobotName(void);
	void readSensors(void);

	/* ground sensor functions */
	bool detectLineCrossroad(void);
	bool detectEndOfLine(void);
	
	/* motor control functions */
	void setWheelSpeedFollowLine(void);
	void setWheelSpeedMoveStraightAhead(void);
	void setWheelSpeedTurnLeft(void);
	void setWheelSpeedTurnRight(void);
	void setWheelSpeedTurnAround(void);
	void applyRobotWheelSpeed(void);
	void performHalt(void);

	/* LED functions */
	void cyclicBlinkingLED(void);
	void allLightsOnLED(void);
	
	/* camera functions */
	void enableCamera(void);
	void disableCamera(void);
	void takeCameraSnapshot(void);
	bool isCameraEnabled(void);

	/* obstacle functions */
	bool detectObstacle(void);

	/* receiver node functions */
	bool getNextReceiverPacket(int* dataPacket);

private:
	/*********************************************************
	* 
	*	internal constants 
	* 
	*********************************************************/ 

	/* motor */
	const static int NO_SIDE = -1;
	const static unsigned int LEFT = 0;
	const static unsigned int RIGHT = 1;
	const double K_GS_SPEED = 0.4;
	const int FORWARD_SPEED = 200;
	const double MOTOR_RATIO = 0.00628;

	const static unsigned int NUM_WHEELS = 2;

	/* IR proximity sensor */
	const static unsigned int NUM_DIST_SENS = 8;
	const static unsigned int PS_RIGHT_00 = 0;
	const static unsigned int PS_RIGHT_45 = 1;
	const static unsigned int PS_RIGHT_90 = 2;
	const static unsigned int PS_RIGHT_REAR = 3;
	const static unsigned int PS_LEFT_REAR = 4;
	const static unsigned int PS_LEFT_90 = 5;
	const static unsigned int PS_LEFT_45 = 6;
	const static unsigned int PS_LEFT_00 = 7;

	/* IR ground color sensor */
	const static unsigned int NUM_GROUND_SENS = 3;
	const static unsigned int GS_BLACK = 310;
	const static unsigned int GS_GROUND = 800;
	const static unsigned int GS_LEFT = 0;
	const static unsigned int GS_CENTER = 1;
	const static unsigned int GS_RIGHT = 2;

	/* LEDs */
	const static unsigned int NB_LEDS = 8;
	const static unsigned int LED_TIME_STEP = 320; // [ms]

	/* obstacle avoidance module */
	const static unsigned int OAM_OBST_THRESHOLD = 100;
	const static unsigned int OAM_FORWARD_SPEED = 150;
	const static unsigned int OAM_K_MAX_DELTAS = 600;
	const double OAM_K_PS_90 = 0.2;
	const double OAM_K_PS_45 = 0.9;
	const double OAM_K_PS_00 = 1.2;

	/* receiver */
	const static unsigned int RECV_SAMPLING_PERIOD = 64;


	/*********************************************************
	*
	*	member variables
	*
	*********************************************************/

	std::string robotName;

	/* robot sensor objects utilized by functions */
	std::array<DistanceSensor*, NUM_DIST_SENS> proximSensors;
	std::array<DistanceSensor*, NUM_GROUND_SENS> groundSensors;
	std::array<Motor*, NUM_WHEELS> motors;
	std::array<LED*, NB_LEDS> robotLEDs;
	Camera* robotCamera;
	Receiver* receiver;

	/* robot's ground sensor values */
	unsigned short gsValues[NUM_GROUND_SENS] = { 0, 0, 0 };

	/* robot's proximity sensor values */
	int psValues[NUM_DIST_SENS] = { 0, 0, 0, 0, 0, 0, 0, 0 };

	/* robot's wheel speed values */
	int wheelSetValue[2];

	/* LED control */
	unsigned int activeLED = 0;
	unsigned int ledTimeCounter = 1;
};

