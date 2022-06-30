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
    robotControl = new RobotControlEPuck(robot);
    pathplanner = new PathPlannerEPuck(robotControl->getRobotName());
    qrmodule = new QRModuleEPuckSGD();
    commWifi = new CommunicationModuleWifi();
        
    /*** get time step from robot routine ***/
    timeStep = robotControl->getTimeStep();

    /* before entering main loop init camera by enabling it */
    robotControl->enableCamera();

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

        /* read current state of sensors */
        robotControl->readSensors();


        /*************************************/
        /********** LED CYCLE BLOCK **********/
        if (endOfLineGoalReached || obstacleDetected /* || crossroadManeuverActive */ )
        {
            robotControl->allLightsOnLED();
        }
        else 
        {
            robotControl->cyclicBlinkingLED();
        }
        /*************************************/
        /*************************************/
        

        /*************************************************/
        /********** CONNECT TO SUPERVISOR BLOCK **********/
        if (!supervisorConnected)
        {            
            if (commWifi->tryToConnectToSupervisor(robotControl->getRobotName())) {

                supervisorConnected = true;
            }
            else {
                std::cerr << robotControl->getRobotName() << ": failed to connect to supervisor." << std::endl;
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

                robotControl->setWheelSpeedTurnAround();
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
                    if (robotControl->isCameraEnabled()) {

                        robotControl->takeCameraSnapshot();
                    }
                    else {
                        /*
                        *   halt, enable camera and wait for 20 steps to 
                        *   ensure camera is fully enabled when taking a snapshot.
                        *   Finish this step of routine afterwards
                        */                          
                        robotControl->setWheelSpeedHalt();
                        robotControl->enableCamera();
                        robotActiveWait(20);
                        continue;
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
                    robotControl->setWheelSpeedFollowLine();
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
            if (crossroadManeuverActive || robotControl->detectLineCrossroad())
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
                            robotControl->setWheelSpeedTurnLeft();
                            crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD;
                        }
                        else if (nextMovingDirection == turnRight) {
                            robotControl->setWheelSpeedTurnRight();
                            crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD;
                        }
                        else if (nextMovingDirection == turnAround) {
                            robotControl->setWheelSpeedTurnAround();
                            crossroadManeuverThreshold = TURNAROUNDTHRESHOLD;
                        }
                        else {
                            /*
                            *   nextMovingDirection == straightOn, configure wheel speed and maneuver threshold.
                            *   when moving straight ahead on a crossroad, use reduced threshold until movement is done
                            */
                            robotControl->setWheelSpeedMoveStraightAhead();
                            crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD / 8;
                        }


                        performingTurn = true;
                    }
                }
            }
            /*************************************/
            /*************************************/


            /**************************************/
            /* OBSTACLE BEHAVIOUR BLOCK */
            /*
            *   check if an obstacle has been detected or if obstacle avoidance
            *   procedure is already activated
            */
            else if (!pathplanner->pathCompleted() && /* border wall of goal position is no obstacle */
                (!obstacleDetected && robotControl->detectObstacle()))
            {
                /* get node parameters from pathplanner module */
                std::tuple<int, int> startNode, goalNode, collisionNode;
                pathplanner->getObstacleParameters(&startNode, &goalNode, &collisionNode);

                /* notify supervisor of collision */
                commWifi->reportCollision(startNode, goalNode, collisionNode);

                /* set robot states for further event handling */
                obstacleDetected = true;
                alternativePathReceived = false; 

                robotControl->setWheelSpeedHalt();

            }
            else if (obstacleDetected) {

                if (!alternativePathReceived) {
                    /*
                    *   obstacle has been detected and collision is reported.
                    *   Wait for supervisor's reply with alternative path.
                    */
                    std::vector<std::tuple<int, int>> path;

                    /* check routine step, check for supervisor response */
                    if (commWifi->receiveCollisionReply(&path)) {
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
                    robotControl->setWheelSpeedTurnAround();

                    turnCounter += timeStep;

                    if (turnCounter >= TURNAROUNDTHRESHOLD)
                    {
                        turnCounter = 0;
                        obstacleDetected = false;
                    }
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
                    }
                    else 
                    {
                        /*
                        *   if no endless mode active, move until end of line and stop
                        */
                        if (robotControl->detectObstacle()) {
                            robotControl->setWheelSpeedHalt();
                            endOfLineGoalReached = true;
                        }
                        else {
                            robotControl->setWheelSpeedFollowLine();
                        }
                    }
                }
                else
                {
                    /*
                    *   following path not completed yet, continue following line
                    */
                    robotControl->setWheelSpeedFollowLine();
                }
            }
            /*************************************/
            /*************************************/
        }

        /*
        *   apply configured speed for each wheel after all states were checked 
        */
        robotControl->applyRobotWheelSpeed();

    };  // END OF MAIN LOOP
    /*************************************/
    /*************************************/

    /*  optional cleanup  */
    delete robot;

    return 0;
}
