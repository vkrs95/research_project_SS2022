#include "RobotRoutine.h"

RobotRoutine::RobotRoutine(Robot* robot)
{
    basicTimeStep = (int)robot->getBasicTimeStep();

    /* Motor initialization */
    motorLeft = robot->getMotor("left wheel motor");
    motorRight = robot->getMotor("right wheel motor");

    motorLeft->setVelocity(0.0);
    motorRight->setVelocity(0.0);

    motorLeft->setPosition(INFINITY);
    motorRight->setPosition(INFINITY);

    lfm_speed[LEFT] = 0;
    lfm_speed[RIGHT] = 0;

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
    for (int i = 0; i < NB_DIST_SENS; i++) 
    {
        sprintf_s(name, "ps%d", i);
        proximSensors[i] = robot->getDistanceSensor(name); /* proximity sensors */
        proximSensors[i]->enable(basicTimeStep);
    }

    /* ground sensor initialization */
    for (int i = 0; i < NB_GROUND_SENS; i++) 
    {
        sprintf_s(name, "gs%d", i);
        groundSensors[i] = robot->getDistanceSensor(name); /* ground sensors */
        groundSensors[i]->enable(basicTimeStep);
    }

    // epuck camera initialisation
    robotCamera = robot->getCamera("camera");
}

void RobotRoutine::ReadSensors()
{
    for (int i = 0; i < NB_DIST_SENS; i++)
        ps_value[i] = (int)proximSensors[i]->getValue();
    for (int i = 0; i < NB_GROUND_SENS; i++)
        gs_value[i] = (int)groundSensors[i]->getValue();
}


void RobotRoutine::LineFollowingModule(void) 
{
    int DeltaS = gs_value[GS_RIGHT] - gs_value[GS_LEFT];

    lfm_speed[LEFT] = LFM_FORWARD_SPEED - LFM_K_GS_SPEED * DeltaS;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED + LFM_K_GS_SPEED * DeltaS;
}


bool RobotRoutine::DetectLineCrossroad(void) 
{
    if (gs_value[GS_LEFT] <= GS_BLACK &&
        gs_value[GS_CENTER] <= GS_BLACK &&
        gs_value[GS_RIGHT] <= GS_BLACK)
        return true;

    return false;
}

bool RobotRoutine::DetectEndOfLine(void) {

    if (gs_value[GS_LEFT] >= GS_GROUND &&
        gs_value[GS_CENTER] >= GS_GROUND &&
        gs_value[GS_RIGHT] >= GS_GROUND)
        return true;

    return false;
}

void RobotRoutine::setWheelSpeedMoveStraightAhead(void)
{
    lfm_speed[LEFT] = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED;
}

void RobotRoutine::setWheelSpeedTurnLeft(void)
{
    lfm_speed[LEFT] = 0;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED;
}

void RobotRoutine::setWheelSpeedTurnRight(void)
{
    lfm_speed[LEFT] = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = 0;
}

void RobotRoutine::setWheelSpeedTurnAround(void)
{
    lfm_speed[LEFT] = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = -LFM_FORWARD_SPEED;
}

void RobotRoutine::CyclicBlinkingLED(void) 
{
    robotLEDs[activeLED]->set(0);
    activeLED = (activeLED + 1) % NB_LEDS;
    robotLEDs[activeLED]->set(1);
}

void RobotRoutine::AllLightsOnLED(void) 
{
    for (int i = 0; i < NB_LEDS; i++) {
        robotLEDs[i]->set(1);
    }
}

void RobotRoutine::SetSpeedAndVelocity(void)
{

    int speed[2] = { 0,0 };

    speed[LEFT] = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];

    motorLeft->setVelocity(0.00628 * speed[LEFT]);
    motorRight->setVelocity(0.00628 * speed[RIGHT]);
}

void RobotRoutine::PerformHalt(void)
{
    // HALT 
    lfm_speed[LEFT] = 0;
    lfm_speed[RIGHT] = 0;

    SetSpeedAndVelocity();
}

void RobotRoutine::EnableEpuckCam(void)
{
    robotCamera->enable(50);
}

void RobotRoutine::DisableEpuckCam(void)
{
    robotCamera->disable();
}

void RobotRoutine::TakeCameraSnapshot(void)
{
    //EnableEpuckCam();
    robotCamera->saveImage(qrImgFileName, 100);
    //DisableEpuckCam();
}

bool RobotRoutine::IsEpuckCamEnabled(void)
{
    // if sampling period is 0, camera is currently disabled
    return robotCamera->getSamplingPeriod() == 0 ? false : true;
}