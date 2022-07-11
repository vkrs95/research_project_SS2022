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
    //robotControl->enableCamera();
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
    *   depending on the current state(s) a speed for each motor is configured in parameter
    *   motorSpeedSetValue of the robotControl object.
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

        /*
        *   #1 step of init procedure:
        *   Scan path information from QR code to later send them to supervisor.
        */
        else if (!qrScanCompleted) {

            if (qrScanHandling() != 0) {
                /* qr scan returned error */
                std::cout << robotControl->getRobotName()
                    << ": error in qr scan procedure. Number of QR scan attempts exceeded." << std::endl;
                return -1;
            }
        }

        /*
        *   #2 step of init procedure: 
        *   Send path information to supervisor and await reply containing shortest 
        *   path to goal. Established connection to supervisor mandatory.
        */
        else if (!initProcedureDone) {

            initPathHandling();
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
            else if (obstacleHandlingActive || robotControl->detectObstacle())
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

int MainControllerEPuck::qrScanHandling(void)
{
    /*
    *   This block is executed when the robot (re-)starts and is facing towards
    *   the border wall of the environment. It shall try to scan a QR code which
    *   is placed at the border wall.
    *   The code should contain start and goal information based on which a path
    *   can be computed.
    */

    /* move towards QR code at start edge node */
    if (initProcedureDistanceToScanCounter >= QR_SCAN_DISTANCE_THRESHOLD) {

        /* check if camera is enabled and take a snapshot via camera while robot is facing towards QR code*/
        if (robotControl->isCameraEnabled()) {

            robotActiveWait(20);
            robotControl->takeCameraSnapshot();
        }
        else {
            /*
            *   halt, enable camera and wait for 20 steps to
            *   ensure camera is fully enabled when taking a snapshot.
            *   Finish this step of routine afterwards
            */
            robotControl->setMotorSpeedHalt();
            robotControl->enableCamera();
            return 0;
        }

        if (qrmodule->readQRCode(robotControl->getQrFileName(), &qrCodeParams)) {

            // deactivate camera since reading was successful
            robotControl->disableCamera();

            mStartNode = qrCodeParams.startXY;
            mGoalNode = qrCodeParams.goalXY;

            qrScanCompleted = true;
        }
        else {

            if (readQrCodeAttemptCounter < QR_READ_ATTEMPTS_THRESHOLD) {
                readQrCodeAttemptCounter++;
                return 0;
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
    

    return 0;
}

void MainControllerEPuck::initPathHandling(void)
{
    /*
    *   This block is executed after the robot successfully read the start and goal
    *   coordinates out of a QR code. These informations are sent to the 
    *   supervisor, awaiting a reply message containing a path. The robot then 
    *   turns 180° to complete the init procedure and is ready to start following 
    *   the line and its calculated path until it has reached its destination.
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
        /* send start + goal information to supervisor */
        if (!pathRequestSent) {
            commModule->requestPath(mStartNode, mGoalNode);
            pathRequestSent = true;
        }
        /* poll and wait for supervisor path reply */
        else {
            /*
            *   Start and goal information has been sent.
            *   Wait for supervisor's reply with path.
            */
            std::vector<std::tuple<int, int>> path;

            if (commModule->receivePath(&path)) {
                pathplanner->setPath(path);
                pathPlanningCompleted = true;
            }
        }
    }
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
            if (commModule->receiveAlternativePath(&path)) {
                pathplanner->setPath(path);
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
        resetControllerStates();
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

void MainControllerEPuck::resetControllerStates(void)
{
    initProcedureDistanceToScanCounter = 0;
    readQrCodeAttemptCounter = 0;

    qrScanCompleted = false;
    pathRequestSent = false;
    pathPlanningCompleted = false;
    initProcedureDone = false;
}