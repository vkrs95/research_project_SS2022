#include "RobotRoutine.h"

RobotRoutine::RobotRoutine(Robot* robot)
{
    basicTimeStep = (int)robot->getBasicTimeStep();

    /* Motor initialization */
    motors[LEFT] = robot->getMotor("left wheel motor");
    motors[RIGHT] = robot->getMotor("right wheel motor");

    motors[LEFT]->setVelocity(0.0);
    motors[RIGHT]->setVelocity(0.0);

    motors[LEFT]->setPosition(INFINITY);
    motors[RIGHT]->setPosition(INFINITY);

    wheelSetValue[LEFT] = 0;
    wheelSetValue[RIGHT] = 0;

    /* receiver initialization */
    receiver = robot->getReceiver("receiver");
    receiver->enable(RECV_SAMPLING_PERIOD);

    // set robot name
    robotName = robot->getName();
    qrImgFileName = "start_goal_" + robotName + ".jpg";

    /* string to parse sensor names in it */
    char name[20];

    /* LED initialization */
    for (int i = 0; i < NB_LEDS; i++) 
    {
        sprintf_s(name, "led%d", i);
        robotLEDs[i] = robot->getLED(name); /* LEDs */
    }

    /* proximitys sensor initialization */
    for (int i = 0; i < NUM_DIST_SENS; i++) 
    {
        sprintf_s(name, "ps%d", i);
        proximSensors[i] = robot->getDistanceSensor(name); /* proximity sensors */
        proximSensors[i]->enable(basicTimeStep);
    }

    /* ground sensor initialization */
    for (int i = 0; i < NUM_GROUND_SENS; i++) 
    {
        sprintf_s(name, "gs%d", i);
        groundSensors[i] = robot->getDistanceSensor(name); /* ground sensors */
        groundSensors[i]->enable(basicTimeStep);
    }

    /* robot camera initialisation */
    robotCamera = robot->getCamera("camera");
}

std::string RobotRoutine::getRobotName(void)
{
    return robotName;
}

void RobotRoutine::readSensors()
{
    for (int i = 0; i < NUM_DIST_SENS; i++)
        psValues[i] = (int)proximSensors[i]->getValue();
    for (int i = 0; i < NUM_GROUND_SENS; i++)
        gsValues[i] = (int)groundSensors[i]->getValue();
}


void RobotRoutine::setWheelSpeedFollowLine(void) 
{
    int DeltaS = gsValues[GS_RIGHT] - gsValues[GS_LEFT];

    wheelSetValue[LEFT] = FORWARD_SPEED - K_GS_SPEED * DeltaS;
    wheelSetValue[RIGHT] = FORWARD_SPEED + K_GS_SPEED * DeltaS;
}


bool RobotRoutine::detectLineCrossroad(void) 
{
    if (gsValues[GS_LEFT] <= GS_BLACK &&
        gsValues[GS_CENTER] <= GS_BLACK &&
        gsValues[GS_RIGHT] <= GS_BLACK)
        return true;

    return false;
}

bool RobotRoutine::detectEndOfLine(void) {

    if (gsValues[GS_LEFT] >= GS_GROUND &&
        gsValues[GS_CENTER] >= GS_GROUND &&
        gsValues[GS_RIGHT] >= GS_GROUND)
        return true;

    return false;
}

void RobotRoutine::setWheelSpeedMoveStraightAhead(void)
{
    wheelSetValue[LEFT] = FORWARD_SPEED;
    wheelSetValue[RIGHT] = FORWARD_SPEED;
}

void RobotRoutine::setWheelSpeedTurnLeft(void)
{
    wheelSetValue[LEFT] = 0;
    wheelSetValue[RIGHT] = FORWARD_SPEED;
}

void RobotRoutine::setWheelSpeedTurnRight(void)
{
    wheelSetValue[LEFT] = FORWARD_SPEED;
    wheelSetValue[RIGHT] = 0;
}

void RobotRoutine::setWheelSpeedTurnAround(void)
{
    wheelSetValue[LEFT] = FORWARD_SPEED;
    wheelSetValue[RIGHT] = -FORWARD_SPEED;
}

void RobotRoutine::cyclicBlinkingLED(void) 
{
    if ((basicTimeStep * ledTimeCounter++) >= LED_TIME_STEP)
    {
        /* disable current LED and enable next one */
        robotLEDs[activeLED]->set(0);
        activeLED = (activeLED + 2) % NB_LEDS;  // increment by 2 to skip RGB LEDs of Epuck V2
        robotLEDs[activeLED]->set(1);
            
        /* reset LED time counter */
        ledTimeCounter = 1;
    }

}

void RobotRoutine::allLightsOnLED(void) 
{
    for (int i = 0; i < NB_LEDS; i++) {
        robotLEDs[i]->set(1);
    }
}

void RobotRoutine::applyRobotWheelSpeed(void)
{
    motors[LEFT]->setVelocity(MOTOR_RATIO * wheelSetValue[LEFT]);
    motors[RIGHT]->setVelocity(MOTOR_RATIO * wheelSetValue[RIGHT]);
}

void RobotRoutine::performHalt(void)
{
    // HALT 
    wheelSetValue[LEFT] = 0;
    wheelSetValue[RIGHT] = 0;

    applyRobotWheelSpeed();
}

void RobotRoutine::enableCamera(void)
{
    robotCamera->enable(50);
}

void RobotRoutine::disableCamera(void)
{
    robotCamera->disable();
}

void RobotRoutine::takeCameraSnapshot(void)
{
    robotCamera->saveImage(qrImgFileName, 100);
}

bool RobotRoutine::isCameraEnabled(void)
{
    // if sampling period is 0, camera is currently disabled
    return robotCamera->getSamplingPeriod() == 0 ? false : true;
}

bool RobotRoutine::detectObstacle(void)
{
    int max_ds_value, DeltaS = 0, i;
    int Activation[] = { 0,0 };

    max_ds_value = 0;

    for (i = RobotRoutine::PS_RIGHT_00; i <= RobotRoutine::PS_RIGHT_45; i++)
    {
        if (max_ds_value < psValues[i]) max_ds_value = psValues[i];
        Activation[RobotRoutine::RIGHT] += psValues[i];
    }

    for (i = RobotRoutine::PS_LEFT_45; i <= RobotRoutine::PS_LEFT_00; i++)
    {
        if (max_ds_value < psValues[i]) max_ds_value = psValues[i];
        Activation[RobotRoutine::LEFT] += psValues[i];
    }

    if (max_ds_value > OAM_OBST_THRESHOLD)
    {
        return true;
    }

    return false;
}

bool RobotRoutine::getNextReceiverPacket(int* dataPacket)
{
    int* localCopy;

    if (receiver->getQueueLength() > 0) {
        localCopy = (int*) receiver->getData();

        if (localCopy != 0) {
            /* 
            *   Received data is valid; try to get next data packet
            */
            memcpy(dataPacket, localCopy, sizeof(int));
            receiver->nextPacket();

            return true;
        }
    }
    receiver->nextPacket();

    /* queue is empty */
    return false;
}