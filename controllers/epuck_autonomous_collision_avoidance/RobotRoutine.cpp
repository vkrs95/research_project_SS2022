#include "RobotRoutine.h"

RobotRoutine::RobotRoutine(Robot* robot)
{
    int timeStep = (int)robot->getBasicTimeStep();
    int i;

    // Motor initialization
    motor_left = robot->getMotor("left wheel motor");
    motor_right = robot->getMotor("right wheel motor");

    motor_left->setVelocity(0.0);
    motor_right->setVelocity(0.0);

    motor_left->setPosition(INFINITY);
    motor_right->setPosition(INFINITY);

    lfm_speed[LEFT] = 0;
    lfm_speed[RIGHT] = 0;
    LFM_FORWARD_SPEED = 200;
    LFM_K_GS_SPEED = 0.4;

    speed[LEFT] = 0;
    speed[RIGHT] = 0;

    // set robot name
    epuck_name = robot->getName();
    qr_img_file_name = "start_goal_" + epuck_name + ".jpg";

    // LED initialization
    for (i = 0; i < NB_LEDS; i++) 
    {
        sprintf_s(name, "led%d", i);
        robot_leds[i] = robot->getLED(name); /* LEDs */
    }

    // Sensor initialization
    for (i = 0; i < NB_DIST_SENS; i++) 
    {
        sprintf_s(name, "ps%d", i);
        proxim_sensors[i] = robot->getDistanceSensor(name); /* proximity sensors */
        proxim_sensors[i]->enable(timeStep);
    }

    for (i = 0; i < NB_GROUND_SENS; i++) 
    {
        sprintf_s(name, "gs%d", i);
        ground_sensors[i] = robot->getDistanceSensor(name); /* ground sensors */
        ground_sensors[i]->enable(timeStep);
    }

    // epuck camera initialisation
    epuck_cam = robot->getCamera("camera");
}

void RobotRoutine::ReadSensors()
{
    for (int i = 0; i < NB_DIST_SENS; i++)
        ps_value[i] = (int)proxim_sensors[i]->getValue();
    for (int i = 0; i < NB_GROUND_SENS; i++)
        gs_value[i] = (int)ground_sensors[i]->getValue();
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

void RobotRoutine::OnCrossroadTurnLeft(void) 
{
    lfm_speed[LEFT] = 0;
    lfm_speed[RIGHT] = LFM_FORWARD_SPEED;
}

void RobotRoutine::OnCrossroadTurnRight(void) 
{
    lfm_speed[LEFT] = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = 0;
}

void RobotRoutine::OnCrossroadTurnDegree(void)
{
    lfm_speed[LEFT] = 200;
    lfm_speed[RIGHT] = -200;
}

void RobotRoutine::CyclicBlinkingLED(void) 
{
    robot_leds[active_led]->set(0);
    active_led = (active_led + 1) % NB_LEDS;
    robot_leds[active_led]->set(1);
}

void RobotRoutine::AllLightsOnLED(void) 
{
    for (int i = 0; i < NB_LEDS; i++) {
        robot_leds[i]->set(1);
    }
}

void RobotRoutine::SetSpeedAndVelocity(void)
{
    speed[LEFT] = lfm_speed[LEFT];
    speed[RIGHT] = lfm_speed[RIGHT];

    motor_left->setVelocity(0.00628 * speed[LEFT]);
    motor_right->setVelocity(0.00628 * speed[RIGHT]);
}

void RobotRoutine::PerformTurnAround(void) {
    lfm_speed[LEFT] = LFM_FORWARD_SPEED;
    lfm_speed[RIGHT] = -LFM_FORWARD_SPEED;
}

void RobotRoutine::PerformHalt(void)
{
    // HALT 
    lfm_speed[LEFT] = 0;
    lfm_speed[RIGHT] = 0;
}

void RobotRoutine::EnableEpuckCam(void)
{
    epuck_cam->enable(50);
}

void RobotRoutine::DisableEpuckCam(void)
{
    epuck_cam->disable();
}

void RobotRoutine::TakeCameraSnapshot(void)
{
    //EnableEpuckCam();
    epuck_cam->saveImage(qr_img_file_name, 100);
    //DisableEpuckCam();
}