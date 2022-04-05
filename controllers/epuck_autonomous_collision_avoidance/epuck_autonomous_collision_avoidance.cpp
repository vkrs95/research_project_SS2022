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
    int groundSensorJitter = 0;
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
    bool getNextDirection = true;
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

    // bool temporary_helper = false;

    /* before entering main loop init camera by enabling it */
    robotroutine->EnableEpuckCam();

    // Main loop:
    while (robot->step(timeStep) != -1) {

        robotroutine->ReadSensors();

        // LED cycle
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

        /********** INIT PROCEDURE **********/
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
        /********** MOVE TO GOAL PROCEDURE *********/
        else if(!endOfLineGoalReached) {

            if (robotroutine->DetectLineCrossroad())
            {
                groundSensorJitter++;
                if (groundSensorJitter > 4 && !performingTurn)
                {
                    robotroutine->AllLightsOnLED();
                    crossroadDetected = true;
                    getNextDirection = true;
                    groundSensorJitter = 0;
                }
            }

            // set flag if a crossroad has been detected
            //if (robotroutine->DetectLineCrossroad())
            //{
            //    performingTurnOnCrossroad = true;
            //}

            if (crossroadDetected)
            {
                //if (pathDirectionListIterator < pathplanner->path_direction_list.size())
                //{
                //    MovingDirection next_direction = pathplanner->path_direction_list.at(pathDirectionListIterator);
                //    if (next_direction == turn_left)
                //        robotroutine->OnCrossroadTurnLeft();
                //    else if (next_direction == turn_right)
                //        robotroutine->OnCrossroadTurnRight();
                //    else
                //    {
                //        robotroutine->lfm_speed[LEFT] = robotroutine->LFM_FORWARD_SPEED;
                //        robotroutine->lfm_speed[RIGHT] = robotroutine->LFM_FORWARD_SPEED;
                //    }

                //    turnCounter += timeStep;

                //    if (turnCounter >= crossroadTurnDone)
                //    {
                //        turnCounter = 0;
                //        performingTurnOnCrossroad = false;
                //        pathDirectionListIterator++;
                //    }
                //}

                if (getNextDirection)
                {
                    AStar::Vec2i predecessor;
                    
                    predecessor = pathplanner->coordinate_list.at(pathIterator - 1);

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

                    getNextDirection = false;
                    performingTurn = true;
                    
                    pathIterator += 2; // because of padding coordinates
                }
                else
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

                    //temporary_helper = true;
                    predPost = pathplanner->coordinate_list[pathIterator];

                    pathplanner->PathPlanning(newWall, startPosition, goalPosition);

                    turnCounter = 0;
                    pathIterator = 1;
                    obstacleDetected = false;
                }
            }

            else if (obstacleavoidance->ObstacleDetection(robotroutine->ps_value))
            {
                //if (temporary_helper)
                //{
                //    robotroutine->LFM_FORWARD_SPEED = 0;
                //    robotroutine->lfm_speed[LEFT] = 0;
                //    robotroutine->lfm_speed[RIGHT] = 0;
                //    avoidSecondCollision = true;
                //}
                //else
                //{
                    robotroutine->AllLightsOnLED();
                    robotroutine->LFM_FORWARD_SPEED = -200;
                    robotroutine->lfm_speed[LEFT] = -200;
                    robotroutine->lfm_speed[RIGHT] = -200;
                    obstacleDetected = true;
//                }
            }

            else
            {
                if (pathplanner->coordinate_list.size() > 0 && // direction command list has been initialized already
                    pathIterator >= pathplanner->coordinate_list.size()) // all direction commands have been executed
                {
                    /********** GOAL REACHED PROCEDURE **********/
                    if (epuckEndlessMode) {

                        // reset variables -> reset_controller_state();
                        startPosition.x = 0; startPosition.y = 0;
                        goalPosition.x = 0; goalPosition.y = 0;
                        pathPlanningCompleted = false;
                        initProcedureDone = false;
                        pathIterator = 1;
                        newWall.clear();

                        robotroutine->EnableEpuckCam();
                    }
                    else {
                        // if no endless mode move until end of line and stop
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
                    robotroutine->LineFollowingModule();
                }
            }
        }

        robotroutine->SetSpeedAndVelocity();
    };

    delete robot;
    return 0;
}
