#include "RobotRoutine.h"
#include "PathPlannerEPuckAStar.h"
#include "ObstacleAvoidance.h"
#include "QRModuleEPuckSGD.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    Robot* robot = new Robot();
    RobotRoutine* robotroutine = new RobotRoutine(robot);
    PathPlannerEPuckAStar* pathplanner = new PathPlannerEPuckAStar();
    ObstacleAvoidance* obstacleavoidance = new ObstacleAvoidance;
    QRModule<SGDQRParams>* qrmodule = new QRModuleEPuckSGD();

    
    /*** define local variables ***/
    int timeStep = (int)robot->getBasicTimeStep();
    unsigned int pathDirectionListIterator = 0;
    const unsigned int CROSSROADTURNTHRESHOLD = 3500;
    const unsigned int TURNAROUNDTHRESHOLD = CROSSROADTURNTHRESHOLD;


    /* robot parameters */
    unsigned int ledCounter = 1;
    unsigned int groundSensorJitter = 0;
    unsigned int groundSensorJitterThreshold = 4;
    bool epuckEndlessMode = true;
    unsigned int qrDistanceToScanPos = 2000;
    unsigned int initProcedureDistanceToScanCounter = 0;


    /* world parameters */
    AStar::Vec2i startPosition;
    AStar::Vec2i goalPosition;

    AStar::Vec2i predPost;
    AStar::CoordinateList newWall;

    SGDQRParams qrCodeParams;


    /* robot routine flags */
    bool obstacleDetected = false;
    bool crossroadDetected = false;
    bool performingTurn = false;
    bool avoidSecondCollision = false;
    bool performingTurnOnCrossroad = false;
    bool endOfLineGoalReached = false;
    unsigned int turnCounter = 0;
    bool initProcedureDone = false;
    bool pathPlanningCompleted = false;
    unsigned int readQrCodeAttemptCounter = 0;
    unsigned int readQrCodeAttemptLimit = 3;

    /* path parameters */
    size_t pathIterator = 1;
    MovingDirection nextMovingDirection;

    /* before entering main loop init camera by enabling it */
    robotroutine->EnableEpuckCam();

    /********** Main Loop **********/
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

        robotroutine->ReadSensors();

        /*************************************/
        /********** LED CYCLE BLOCK **********/
        if (endOfLineGoalReached || obstacleDetected /* || crossroadDetected */ )
        {
            robotroutine->AllLightsOnLED();
        }
        else 
        {
            if ((timeStep * ledCounter++) >= ledTimeStep)
            {
                robotroutine->CyclicBlinkingLED();
                ledCounter = 1;
            }
        }

        /*************************************/
        /*************************************/


        /*************************************/
        /******* INIT PROCEDURE BLOCK ********/
        if (!initProcedureDone) {

            if (pathPlanningCompleted) {

                robotroutine->PerformTurnAround();
                turnCounter += timeStep;

                if (turnCounter >= TURNAROUNDTHRESHOLD) {
                    turnCounter = 0;
                    initProcedureDone = true;
                }
            }
            else {
                if (initProcedureDistanceToScanCounter >= qrDistanceToScanPos) {

                    /* check if camera is enabled and take a snapshot via camera while robot is facing towards QR code*/
                    if (robotroutine->IsEpuckCamEnabled()) {

                        robotroutine->PerformHalt();
                        robotroutine->TakeCameraSnapshot();
                    }
                    else {
                        // enable camera and finish this step of routine 
                        robotroutine->EnableEpuckCam();
                        continue;
                    }

                    // get content from QR image
                    bool readSuccessful = qrmodule->readQRCode(robotroutine->qr_img_file_name, &qrCodeParams);

                    if (readSuccessful) {

                        // deactivate epuck camera since reading was successful
                        robotroutine->DisableEpuckCam();

                        pathplanner->setMatrixDimension(qrCodeParams.mapDimension);
                        
                        // save positions read from QR code
                        startPosition = pathplanner->MPList.at(qrCodeParams.startIndex);
                        goalPosition = pathplanner->MPList.at(qrCodeParams.goalIndex);

                        // debug output of start/goal information
                        std::cout << "------------------------" << "\n";
                        std::cout << "E-Puck: " << robotroutine->epuck_name << " in " << qrCodeParams.mapDimension << "x" << qrCodeParams.mapDimension << " map\n";
                        std::cout << "Start Position: P" << qrCodeParams.startIndex + 1 << "(" << startPosition.x << ", " << startPosition.y <<
                            ")\nGoal Position: P" << qrCodeParams.goalIndex + 1 << "(" << goalPosition.x << ", " << goalPosition.y << ")" << "\n";
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

                    /* path planning */
                    pathplanner->findPath(startPosition, goalPosition);

                    initProcedureDistanceToScanCounter = 0;
                    pathPlanningCompleted = true;
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
        else if(!endOfLineGoalReached) {

            /*
            *   while following the line check if one or more states occure and
            *   execute the corresponding behaviour   
            */

            if (robotroutine->DetectLineCrossroad())
            {
                /*
                *   in case a crossroad has been detected increase groundSensorJitter
                *   until a threshold is exceeded. This is necessary to avoid false detection 
                *   of a crossroad due to a sensor flaw
                */
                groundSensorJitter++;
                if (groundSensorJitter > groundSensorJitterThreshold && !performingTurn)
                {
                    /*
                    *   crossroad detection valid, load current position from coordinate list
                    *   by setting predecessor, current position and successor;
                    *   set flag to continue performing turn on crossroad
                    */
                    crossroadDetected = true;
                    groundSensorJitter = 0;

                    /*
                    *   get next moving direction to perform on the crossroad
                    */
                    nextMovingDirection = pathplanner->getNextMovingDirection(pathIterator);

                    /*
                    *   configure wheel speed according to moving direction
                    */
                    if (nextMovingDirection == turnLeft)           robotroutine->setWheelSpeedTurnLeft();
                    else if (nextMovingDirection == turnRight)     robotroutine->setWheelSpeedTurnRight();
                    else /* nextMovingDirection == straightOn */   robotroutine->setWheelSpeedMoveStraightAhead();


                    performingTurn = true;
                    pathIterator += 2; // because of padding coordinates
                }
            }

            if (crossroadDetected)
            {                
                turnCounter += timeStep;

                /* turn maneuver should be finished after turn counter has exceeded threshold */
                if (nextMovingDirection == straightOn) {
                    /* when moving straight ahead on a crossroad, use reduced threshold until movement is done */
                    if (turnCounter >= CROSSROADTURNTHRESHOLD/2)
                    {
                        turnCounter = 0;
                        crossroadDetected = false;
                        performingTurn = false;
                    }
                }
                else {
                    if (turnCounter >= CROSSROADTURNTHRESHOLD)
                    {
                        turnCounter = 0;
                        crossroadDetected = false;
                        performingTurn = false;
                    }
                }
            }

            /*
            *   check if an obstacle has been detected or if obstacle avoidance
            *   procedure is already activated
            */
            else if (obstacleDetected || obstacleavoidance->ObstacleDetection(robotroutine->ps_value))
            {
                obstacleDetected = true;

                robotroutine->setWheelSpeedTurnAround();

                turnCounter += timeStep;

                if (turnCounter >= TURNAROUNDTHRESHOLD)
                {
                    newWall.push_back(pathplanner->coordinateList[pathIterator]);
                    startPosition = pathplanner->coordinateList[pathIterator - 1];

                    predPost = pathplanner->coordinateList[pathIterator];

                    pathplanner->findPath(newWall, startPosition, goalPosition);

                    turnCounter = 0;
                    pathIterator = 1;
                    obstacleDetected = false;
                }
            }

            
            /*************************************/
            /****** GOAL REACHED PROCEDURE *******/
            else
            {
                /* 
                *   check if state 'goal reached' is reached
                */
                if (pathplanner->coordinateList.size() > 0 && // direction command list has been initialized already
                    pathIterator >= pathplanner->coordinateList.size()) // all direction commands have been executed
                {                    
                    if (epuckEndlessMode) 
                    {
                        /*
                        *   endless mode behavior
                        */
                        // reset variables -> reset_controller_state();
                        startPosition.x = 0; startPosition.y = 0;
                        goalPosition.x = 0; goalPosition.y = 0;
                        pathPlanningCompleted = false;
                        initProcedureDone = false;
                        pathIterator = 1;
                        newWall.clear();

                        robotroutine->EnableEpuckCam();
                    }
                    else 
                    {
                        /*
                        *   if no endless mode active, move until end of line and stop
                        */
                        if (robotroutine->DetectEndOfLine()) {
                            if (groundSensorJitter >= 10) {
                                robotroutine->PerformHalt();
                                endOfLineGoalReached = true;
                                groundSensorJitter = 0;
                            }
                            groundSensorJitter++;
                        }
                        else {
                            robotroutine->LineFollowingModule();
                        }
                    }
                }
                else
                {
                    /*
                    *   coordinate list not empty --> goal not reached, continue following line
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

    /*  optional cleanup  */
    delete robot;

    return 0;
}
