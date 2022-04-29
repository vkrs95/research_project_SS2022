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

    motorLeft->setVelocity(MOTOR_RATIO * speed[LEFT]);
    motorRight->setVelocity(MOTOR_RATIO * speed[RIGHT]);
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

bool RobotRoutine::detectObstacle(void)
{
    int max_ds_value, DeltaS = 0, i;
    int Activation[] = { 0,0 };

    max_ds_value = 0;

    for (i = RobotRoutine::PS_RIGHT_00; i <= RobotRoutine::PS_RIGHT_45; i++)
    {
        if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
        Activation[RobotRoutine::RIGHT] += ps_value[i];
    }

    for (i = RobotRoutine::PS_LEFT_45; i <= RobotRoutine::PS_LEFT_00; i++)
    {
        if (max_ds_value < ps_value[i]) max_ds_value = ps_value[i];
        Activation[RobotRoutine::LEFT] += ps_value[i];
    }

    if (max_ds_value > OAM_OBST_THRESHOLD)
    {
        return true;
    }

    return false;
}