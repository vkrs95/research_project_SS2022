#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>
#include <webots/Receiver.hpp>

#pragma once


// All the webots classes are defined in the "webots" namespace
using namespace webots;

class IRobotControl
{

public:
	// Empty virtual destructor for proper cleanup
	virtual ~IRobotControl() {}

	/*********************************************************
	*
	*	public functions provided by robot routine module
	*
	*********************************************************/
		
	/* general functions */
	virtual int getTimeStep(void) = 0;
	virtual std::string getRobotName(void) = 0;
	virtual std::string getQrFileName(void) = 0;
	virtual void readSensors(void) = 0;

	/* ground sensor functions */
	virtual bool detectLineCrossroad(void) = 0;
	virtual bool detectEndOfLine(void) = 0;
	
	/* basic motor control functions */
	virtual void setMotorSpeedFollowLine(void) = 0;
	virtual void setMotorSpeedMoveStraightAhead(void) = 0;
	virtual void setMotorSpeedTurnLeft(void) = 0;
	virtual void setMotorSpeedTurnRight(void) = 0;
	virtual void setMotorSpeedTurnAround(void) = 0;
	virtual void setMotorSpeedHalt(void) = 0;
	virtual void applyRobotMotorSpeed(void) = 0;

	/* LED functions */
	virtual void cyclicBlinkingLED(void) = 0;
	virtual void allLightsOnLED(void) = 0;
	
	/* camera functions */
	virtual void enableCamera(void) = 0;
	virtual void disableCamera(void) = 0;
	virtual void takeCameraSnapshot(void) = 0;
	virtual bool isCameraEnabled(void) = 0;

	/* obstacle functions */
	virtual bool detectObstacle(void) = 0;

	/* receiver node functions */
	virtual bool getNextReceiverPacket(int* dataPacket) = 0;

};
