#include "epuck_autonomous_collision_avoidance.h"

void robotActiveWait(int numOfSteps)
{
    for (int i = 0; i < numOfSteps; i++) {
        if (robot->step(timeStep) == -1)
            break;
    }
}

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    robotroutine = new RobotRoutine(robot);
    pathplanner = new PathPlannerEPuck(robotroutine->robotName);
    qrmodule = new QRModuleEPuckSGD();
    commWifi = new CommunicationModuleWifi();
        
    /*** get time step from robot routine ***/
    timeStep = robotroutine->basicTimeStep;

    /* before entering main loop init camera by enabling it */
    robotroutine->EnableEpuckCam();

    /*************************************/
    /************* MAIN LOOP *************/
    /*
    *   During the lifetime of the robot the controller main loop is executed periodically.
    *   The loop is constructed as a state machine.
    *   In every loop execution a list of state conditions is checked and depending on the current
    *   environment/actions/states different behaviour blocks are executed. 
    *   
    *   Moving through environment: 
    *   depending on the current state(s) a speed for each wheel is configured in parameter
    *   lfm_speed of the robotroutine object. 
    *   At the end of each main loop pass the preconfigured speed is then applied to the robot
    * 
    */
    while (robot->step(timeStep) != -1) {

        /*   if (fd == 0) {
            fd = commWifi->socket_accept(sfd);
            if (fd > 0)
                commWifi->socket_set_non_blocking(fd);
            else if (fd < 0)
                ;
        }
        */
        robotroutine->ReadSensors();

        /*************************************/
        /********** LED CYCLE BLOCK **********/
        if (endOfLineGoalReached || obstacleDetected /* || crossroadManeuverActive */ )
        {
            robotroutine->AllLightsOnLED();
        }
        else 
        {
            if ((timeStep * ledCounter++) >= RobotRoutine::LED_TIME_STEP)
            {
                robotroutine->CyclicBlinkingLED();
                ledCounter = 1;
            }
        }
        /*************************************/
        /*************************************/
        

        /*************************************************/
        /********** CONNECT TO SUPERVISOR BLOCK **********/
        if (!supervisorConnected)
        {            
            if (commWifi->tryToConnectToSupervisor(robotroutine->robotName)) {

                supervisorConnected = true;
            }            
        }
        /*************************************/
        /*************************************/


        /*************************************/
        /******* INIT PROCEDURE BLOCK ********/
        /*
        *   This block is executed when the robot (re-)starts and is facing towards 
        *   the border wall of the environment. It shall try to scan a QR code which 
        *   is placed at the border wall. 
        *   The code should contain start and goal information based on which a path
        *   can be computed. The robot then turns 180° and starts following the line and
        *   its calculated path until it has reached its destination.
        */
        if (!initProcedureDone) {

            if (pathPlanningCompleted) {

                robotroutine->setWheelSpeedTurnAround();
                turnCounter += timeStep;

                if (turnCounter >= TURNAROUNDTHRESHOLD) {
                    turnCounter = 0;
                    initProcedureDone = true;
                }
            }
            else {
                /* move towards QR code at start edge node */
                if (initProcedureDistanceToScanCounter >= qrDistanceToScanPos) {

                    /* check if camera is enabled and take a snapshot via camera while robot is facing towards QR code*/
                    if (robotroutine->IsEpuckCamEnabled()) {

                        robotroutine->TakeCameraSnapshot();
                    }
                    else {
                        /*
                        *   halt, enable camera and wait for 20 steps to 
                        *   ensure camera is fully enabled when taking a snapshot.
                        *   Finish this step of routine afterwards
                        */                          
                        robotroutine->PerformHalt();
                        robotroutine->EnableEpuckCam();
                        robotActiveWait(20);
                        continue;
                    }

                    // generate structure to save read parameters into
                    SGDQRParams qrCodeParams;

                    // get content from QR image
                    bool readSuccessful = qrmodule->readQRCode(robotroutine->qrImgFileName, &qrCodeParams);

                    if (readSuccessful) {

                        // deactivate epuck camera since reading was successful
                        robotroutine->DisableEpuckCam();

                        // save parameters read from QR code
                        pathplanner->setMatrixDimension(qrCodeParams.mapDimension);
                        pathplanner->setStartGoalPositionByIndex(qrCodeParams.startIndex, qrCodeParams.goalIndex);
                        
                        /* 
                        *   do path planning, no parameters necessary since start and goal are 
                        *   known by pathplanner through call of setStartGoalPositionByIndex() 
                        */
                        pathplanner->findPath();

                        initProcedureDistanceToScanCounter = 0;
                        pathPlanningCompleted = true;
                    }
                    else {
                        
                        if (readQrCodeAttemptCounter < readQrCodeAttemptLimit) {
                            readQrCodeAttemptCounter++;
                            continue; // finish this routine step 
                        }
                        else {
                            // number of attempts exceeded, handle error
                            endOfLineGoalReached = true;
                            return -1;
                        }
                    }
                }
                else {
                    robotroutine->LineFollowingModule();
                    initProcedureDistanceToScanCounter += timeStep;
                }
            }
        }
        /*************************************/
        /*************************************/


        /*************************************/
        /**** MOVE TO GOAL PROCEDURE BLOCK ***/
        /*
        *   while following the line check if one or more states occure and
        *   execute the corresponding behaviour
        */
        else if(!endOfLineGoalReached && initProcedureDone) {


            /*************************************/
            /***** CROSSROAD MANEUVER BLOCK ******/
            /*
            *   check if the ground sensors of the robot have detected a crossroad or
            *   if the crossroad maneuver behaviour is already active
            */
            if (crossroadManeuverActive || robotroutine->DetectLineCrossroad())
            {
                if (crossroadManeuverActive) {
                    /*
                    *   A crossroad has been detected and the turn maneuver is activated.
                    *   Repeat executing this state until maneuver is performed
                    */
                    turnCounter += timeStep;

                    /* turn maneuver should be finished after turn counter has exceeded threshold */
                    if (turnCounter >= crossroadManeuverThreshold)
                    {
                        turnCounter = 0;
                        crossroadManeuverActive = false;
                        performingTurn = false;
                    }
                }
                else {
                    /*
                    *   Crossroad maneuver is not active yet. However, a crossroad has been 
                    *   detected by the ground sensors. Increase groundSensorJitter until a 
                    *   threshold is exceeded. This is necessary to avoid false detection of 
                    *   a crossroad due to sensor measurement errors
                    */
                    groundSensorJitter++;
                    if (groundSensorJitter > groundSensorJitterThreshold && !performingTurn)
                    {
                        /*
                        *   crossroad detection valid, load current position from coordinate list
                        *   by setting predecessor, current position and successor;
                        *   set flag to continue performing turn on crossroad
                        */
                        crossroadManeuverActive = true;
                        groundSensorJitter = 0;

                        /*
                        *   get next moving direction to perform on the crossroad
                        */
                        MovingDirection nextMovingDirection = pathplanner->getNextMovingDirection();

                        /*
                        *   configure wheel speed and maneuver threshold according to moving direction
                        */
                        if (nextMovingDirection == turnLeft) {
                            robotroutine->setWheelSpeedTurnLeft();
                            crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD;
                        }
                        else if (nextMovingDirection == turnRight) {
                            robotroutine->setWheelSpeedTurnRight();
                            crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD;
                        }
                        else {
                            /*
                            *   nextMovingDirection == straightOn, configure wheel speed and maneuver threshold.
                            *   when moving straight ahead on a crossroad, use reduced threshold until movement is done
                            */
                            robotroutine->setWheelSpeedMoveStraightAhead();
                            crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD / 8;
                        }


                        performingTurn = true;
                    }
                }
            }
            /*************************************/
            /*************************************/


            /**************************************/
            /* OBSTACLE AVOIDANCE PROCEDURE BLOCK */
            /*
            *   check if an obstacle has been detected or if obstacle avoidance
            *   procedure is already activated
            */
            else if (!pathplanner->pathCompleted() && /* border wall of goal position is no obstacle */
                (obstacleDetected || robotroutine->detectObstacle()) )
            {
                obstacleDetected = true;

                robotroutine->setWheelSpeedTurnAround();

                turnCounter += timeStep;

                if (turnCounter >= TURNAROUNDTHRESHOLD)
                {
                    /* find an alternative path to circumnavigate detected obstacle */
                    pathplanner->findAlternativePath();

                    turnCounter = 0;
                    obstacleDetected = false;
                }
            }
            /*************************************/
            /*************************************/
            

            /*************************************/
            /****** GOAL REACHED PROCEDURE *******/
            else
            {
                /* 
                *   check if state 'goal reached' is reached
                */
                if (pathplanner->pathCompleted()) // all direction commands have been executed
                {                    
                    if (epuckEndlessMode) 
                    {
                        /*
                        *   endless mode behavior
                        */
                        // reset variables -> reset_controller_state();
                        pathPlanningCompleted = false;
                        initProcedureDone = false;

                        // robotroutine->EnableEpuckCam();
                    }
                    else 
                    {
                        /*
                        *   if no endless mode active, move until end of line and stop
                        */
                        if (robotroutine->detectObstacle()) {
                            robotroutine->PerformHalt();
                            endOfLineGoalReached = true;
                        }
                        else {
                            robotroutine->LineFollowingModule();
                        }
                    }
                }
                else
                {
                    /*
                    *   following path not completed yet, continue following line
                    */
                    robotroutine->LineFollowingModule();
                }
            }
            /*************************************/
            /*************************************/
        }

        /*
        *   apply configured speed for each wheel after all states were checked 
        */
        robotroutine->SetSpeedAndVelocity();

    };  // END OF MAIN LOOP
    /*************************************/
    /*************************************/

    /*  optional cleanup  */
    delete robot;

    return 0;
}
