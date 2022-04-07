#include "RobotRoutine.h"
#include "Pathplanner.h"
#include "ObstacleAvoidance.h"
#include "QRModuleEPuckSGD.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    Robot* robot = new Robot();
    RobotRoutine* robotroutine = new RobotRoutine(robot);
    Pathplanner* pathplanner = new Pathplanner();
    ObstacleAvoidance* obstacleavoidance = new ObstacleAvoidance;
    QRModule<SGDQRParams>* qrmodule = new QRModuleEPuckSGD();

    
    /*** define local variables ***/
    int timeStep = (int)robot->getBasicTimeStep();
    unsigned int pathDirectionListIterator = 0;
    unsigned int crossroadTurnDone = 3500;
    unsigned int turnAroundDone = crossroadTurnDone;


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
    unsigned int pathIterator = 1;    
    MovingDirection currentDirection;

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
        if (endOfLineGoalReached) 
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

                if (turnCounter >= turnAroundDone) {
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

                        pathplanner->SetMatrixDimension(qrCodeParams.mapDimension);

                        // save positions read from QR code
                        startPosition = pathplanner->MP_List.at(qrCodeParams.startIndex);
                        goalPosition = pathplanner->MP_List.at(qrCodeParams.goalIndex);

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
                    pathplanner->PathPlanning(startPosition, goalPosition);

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
                    robotroutine->AllLightsOnLED();
                    crossroadDetected = true;
                    groundSensorJitter = 0;



                    AStar::Vec2i predecessor = pathplanner->coordinate_list.at(pathIterator - 1);
                    AStar::Vec2i current = pathplanner->coordinate_list.at(pathIterator);
                    AStar::Vec2i successor = pathplanner->coordinate_list.at(pathIterator + 1);

                    currentDirection = pathplanner->EvaluateDirection(predecessor, current, successor);

                    /*std::cout << "Epuck:" << "( " << robot->getName() << " )\n";
                    std::cout << "Pathplanning in iteration:" << "( " << pathIterator << " )\n";
                    std::cout << "New direction (0 = Straight, 1 = left, 2 = right)" << "(" << currentDirection << ")\n";
                    std::cout << "Predecessor P" << "(" << predecessor.x << ", " << predecessor.y << ")\n";
                    std::cout << "Current P" << "(" << current.x << ", " << current.y << ")\n";
                    std::cout << "Successor P" << "(" << successor.x << ", " << successor.y << ")\n";
                    std::cout << "---------------------------------------\n";*/

                    performingTurn = true;

                    pathIterator += 2; // because of padding coordinates
                }
            }

            if (crossroadDetected)
            {                
                /* set speed of each wheel depening on executing movement */
                if (currentDirection == turn_left)
                    robotroutine->OnCrossroadTurnLeft();
                else if (currentDirection == turn_right)
                    robotroutine->OnCrossroadTurnRight();
                else
                {
                    robotroutine->lfm_speed[LEFT] = robotroutine->LFM_FORWARD_SPEED;
                    robotroutine->lfm_speed[RIGHT] = robotroutine->LFM_FORWARD_SPEED;
                }

                turnCounter += timeStep;

                /* turn maneuver should be finished after turn counter has exceeded threshold */
                if (currentDirection == straight_on) {
                    /* when moving straight ahead on a crossroad, use reduced threshold until movement is done */
                    if (turnCounter >= crossroadTurnDone/2)
                    {
                        turnCounter = 0;
                        crossroadDetected = false;
                        performingTurn = false;
                    }
                }
                else {
                    if (turnCounter >= crossroadTurnDone)
                    {
                        turnCounter = 0;
                        crossroadDetected = false;
                        performingTurn = false;
                    }
                }
            }

            else if (obstacleDetected)
            {
                robotroutine->LFM_FORWARD_SPEED = 200;
                robotroutine->OnCrossroadTurnDegree();

                turnCounter += timeStep;

                if (turnCounter >= crossroadTurnDone)
                {
                    newWall.push_back(pathplanner->coordinate_list[pathIterator]);
                    startPosition = pathplanner->coordinate_list[pathIterator - 1];

                    predPost = pathplanner->coordinate_list[pathIterator];

                    pathplanner->PathPlanning(newWall, startPosition, goalPosition);

                    turnCounter = 0;
                    pathIterator = 1;
                    obstacleDetected = false;
                }
            }

            else if (obstacleavoidance->ObstacleDetection(robotroutine->ps_value))
            {
                robotroutine->AllLightsOnLED();
                robotroutine->LFM_FORWARD_SPEED = -200;
                robotroutine->lfm_speed[LEFT] = -200;
                robotroutine->lfm_speed[RIGHT] = -200;
                obstacleDetected = true;
            }
            
            /*************************************/
            /****** GOAL REACHED PROCEDURE *******/
            else
            {
                /* 
                *   check if state 'goal reached' is reached
                */
                if (pathplanner->coordinate_list.size() > 0 && // direction command list has been initialized already
                    pathIterator >= pathplanner->coordinate_list.size()) // all direction commands have been executed
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
