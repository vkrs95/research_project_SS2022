#include "MainControllerEPuck.h"

MainControllerEPuck::MainControllerEPuck(void)
{
    /*** create all Object instances ***/
    robot = new Robot();
    robotControl = new RobotControlEPuck(robot);
    pathplanner = new PathPlannerEPuck(robotControl->getRobotName());
    qrmodule = new QRModuleEPuckSGD();
    commModule = new CommModuleTCP();

    /*** get time step from robot routine ***/
    timeStep = robotControl->getTimeStep();

    /* before entering main loop init camera by enabling it */
    robotControl->enableCamera();
}

int MainControllerEPuck::mainControllerRoutine(int argc, char** argv)
{
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
    *   wheelSetValue of the robotControl object.
    *   At the end of each main loop pass the preconfigured speed is then applied to the robot
    *
    */
    while (robot->step(timeStep) != -1) {

        /* every loop iteration read current states of sensors */
        robotControl->readSensors();

        /* set LEDs depending of state */
        controlLEDHandling();

        /*
        *   if not done yet the robot is supposed to connect to the 
        *   supervisor entity in order to exchange path information 
        *   and notify collisions
        */
        if (!supervisorConnected)
        {
            if (commModule->registerAtSupervisor(robotControl->getRobotName())) {

                supervisorConnected = true;
            }
            else {
                std::cerr << robotControl->getRobotName() << ": failed to connect to supervisor." << std::endl;
            }
        }
        
        if (!initProcedureDone) {

            if (initProcedureHandling() > 0) {
                /* init procedure expects to finish this routine step */
                continue;
            }
        }

        /*
        *   while following the grid line, check if one or more states occure and
        *   execute the corresponding behaviour
        */
        else if (!endOfLineGoalReached && initProcedureDone) {

            /*
            *   check if the ground sensors of the robot have detected a crossroad or
            *   if the crossroad maneuver behaviour is already active
            */
            if (crossroadManeuverActive || robotControl->detectLineCrossroad())
            {
                crossroadDetectionHandling();
            }

            /* 
            *   check if an obstacle has been detected by the proximity sensors
            *   or if obstacle handling flag is already set
            */
            else if (robotControl->detectObstacle() || obstacleHandlingActive)
            {
                obstacleHandling();
            }
            
            /* 
            *   default behaviour after all other states have been checked:
            *   either robot has reached its goal or it is still following its 
            *   path and moving along the line 
            */
            else
            {
                /* check if state 'goal reached' is reached */
                if (pathplanner->pathCompleted()) // all direction commands have been executed
                {
                    goalReachedHandling();
                }
                else
                {
                    /* following path not completed yet, continue following line */
                    robotControl->setMotorSpeedFollowLine();
                }
            }
        }

        /* apply configured speed for each wheel after all states were checked */
        robotControl->applyRobotMotorSpeed();

    };  // END OF MAIN LOOP

    /*  optional cleanup  */
    delete robot;

    return 0;
}

void MainControllerEPuck::robotActiveWait(int numOfSteps)
{
    for (int i = 0; i < numOfSteps; i++) {
        if (robot->step(timeStep) == -1)
            break;
    }
}

void MainControllerEPuck::controlLEDHandling(void)
{
    if (endOfLineGoalReached || obstacleHandlingActive /* || crossroadManeuverActive */)
    {
        robotControl->allLightsOnLED();
    }
    else
    {
        robotControl->cyclicBlinkingLED();
    }
}

int MainControllerEPuck::initProcedureHandling(void)
{
    /*
    *   This block is executed when the robot (re-)starts and is facing towards
    *   the border wall of the environment. It shall try to scan a QR code which
    *   is placed at the border wall.
    *   The code should contain start and goal information based on which a path
    *   can be computed. The robot then turns 180° and starts following the line and
    *   its calculated path until it has reached its destination.
    */
    if (pathPlanningCompleted) {

        robotControl->setMotorSpeedTurnAround();
        turnCounter += timeStep;

        if (turnCounter >= TURN_AROUND_THRESHOLD) {
            turnCounter = 0;
            initProcedureDone = true;
        }
    }
    else {
        /* move towards QR code at start edge node */
        if (initProcedureDistanceToScanCounter >= QR_SCAN_DISTANCE_THRESHOLD) {

            /* check if camera is enabled and take a snapshot via camera while robot is facing towards QR code*/
            if (robotControl->isCameraEnabled()) {

                robotControl->takeCameraSnapshot();
            }
            else {
                /*
                *   halt, enable camera and wait for 20 steps to
                *   ensure camera is fully enabled when taking a snapshot.
                *   Finish this step of routine afterwards
                */
                robotControl->setMotorSpeedHalt();
                robotControl->applyRobotMotorSpeed();
                robotControl->enableCamera();
                robotActiveWait(20);
                return 1;
            }

            // generate structure to save read parameters into
            SGDQRParams qrCodeParams;

            // get content from QR image
            bool readSuccessful = qrmodule->readQRCode(robotControl->getQrFileName(), &qrCodeParams);

            if (readSuccessful) {

                // deactivate camera since reading was successful
                robotControl->disableCamera();

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

                if (readQrCodeAttemptCounter < QR_READ_ATTEMPTS_THRESHOLD) {
                    readQrCodeAttemptCounter++;
                    return 1; // finish this routine step 
                }
                else {
                    // number of attempts exceeded, handle error
                    endOfLineGoalReached = true;
                    return -1;
                }
            }
        }
        else {
            robotControl->setMotorSpeedFollowLine();
            initProcedureDistanceToScanCounter += timeStep;
        }
    }

    return 0;
}

void MainControllerEPuck::crossroadDetectionHandling(void)
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
        if (groundSensorJitter > GROUND_SENSOR_JITTER_THRESHOLD && !performingTurn)
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
                robotControl->setMotorSpeedTurnLeft();
                crossroadManeuverThreshold = TURN_LEFT_RIGHT_THRESHOLD;
            }
            else if (nextMovingDirection == turnRight) {
                robotControl->setMotorSpeedTurnRight();
                crossroadManeuverThreshold = TURN_LEFT_RIGHT_THRESHOLD;
            }
            else if (nextMovingDirection == turnAround) {
                robotControl->setMotorSpeedTurnAround();
                crossroadManeuverThreshold = TURN_AROUND_THRESHOLD;
            }
            else {
                /*
                *   nextMovingDirection == straightOn, configure wheel speed and maneuver threshold.
                *   when moving straight ahead on a crossroad, use reduced threshold until movement is done
                */
                robotControl->setMotorSpeedMoveStraightAhead();
                crossroadManeuverThreshold = TURN_LEFT_RIGHT_THRESHOLD / 8;
            }


            performingTurn = true;
        }
    }
}

void MainControllerEPuck::obstacleHandling(void)
{
    /*
    *   continue with handling obstacle behaviour if flag
    */
    if (obstacleHandlingActive)
    {
        if (!alternativePathReceived) {
            /*
            *   obstacle has been detected and collision is reported.
            *   Wait for supervisor's reply with alternative path.
            */
            std::vector<std::tuple<int, int>> path;

            /* check routine step, check for supervisor response */
            if (commModule->receiveCollisionReply(&path)) {
                pathplanner->runAlternativePath(path);
                alternativePathReceived = true;
            }
            else { /* wait some time ? ... */ }
        }
        else {
            /*
            *   Supervisor replied with alternative path. Turn around, move to predecessor node and
            *   continue following alternative path
            */
            robotControl->setMotorSpeedTurnAround();

            turnCounter += timeStep;

            if (turnCounter >= TURN_AROUND_THRESHOLD)
            {
                turnCounter = 0;
                obstacleHandlingActive = false;
            }
        }
    }
    /*
    *   check if obstacle handling is not active yet or if
    *   obstacle detection was triggered by border wall of goal position
    */
    else if (!obstacleHandlingActive && !pathplanner->pathCompleted())
    {
        /* get node parameters from pathplanner module */
        std::tuple<int, int> startNode, goalNode, collisionNode;
        pathplanner->getObstacleParameters(&startNode, &goalNode, &collisionNode);

        /* notify supervisor of collision */
        commModule->reportCollision(startNode, goalNode, collisionNode);

        /* set robot states for further event handling */
        obstacleHandlingActive = true;
        alternativePathReceived = false;

        robotControl->setMotorSpeedHalt();

    }
    else {
        /*
        *   obstacle detection has been triggered while state of robot
        *   path is marked as completed.
        */

        // TODO: further handling ?
    }
}

void MainControllerEPuck::goalReachedHandling(void)
{
    if (endlessMode)
    {
        /*
        *   endless mode behavior
        */
        // reset variables -> reset_controller_state();
        pathPlanningCompleted = false;
        initProcedureDone = false;
    }
    else
    {
        /*
        *   if no endless mode active, move until end of line and stop
        */
        if (robotControl->detectObstacle()) {
            /* detected obstacle should be border wall */
            robotControl->setMotorSpeedHalt();
            endOfLineGoalReached = true;
        }
        else {
            /* no wall detected yet, continue following line */
            robotControl->setMotorSpeedFollowLine();
        }
    }
}
