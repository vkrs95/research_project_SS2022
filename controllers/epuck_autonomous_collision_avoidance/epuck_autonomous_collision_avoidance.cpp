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
    unsigned int crossroadManeuverThreshold;    // is set when next direction is read  
    const unsigned int TURNLEFTRIGHTTHRESHOLD = 3500;
    const unsigned int TURNAROUNDTHRESHOLD = TURNLEFTRIGHTTHRESHOLD;


    /* robot parameters */
    unsigned int ledCounter = 1;
    unsigned int groundSensorJitter = 0;
    unsigned int groundSensorJitterThreshold = 4;
    bool epuckEndlessMode = true;
    unsigned int qrDistanceToScanPos = 2000;
    unsigned int initProcedureDistanceToScanCounter = 0;


    /* world parameters */
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

                        // save parameters read from QR code
                        pathplanner->setMatrixDimension(qrCodeParams.mapDimension);
                        pathplanner->setStartGoalPositionByIndex(qrCodeParams.startIndex, qrCodeParams.goalIndex);
                        
                        //// debug output of start/goal information
                        //std::cout << "------------------------" << "\n";
                        //std::cout << "E-Puck: " << robotroutine->epuck_name << " in " << qrCodeParams.mapDimension << "x" << qrCodeParams.mapDimension << " map\n";
                        //std::cout << "Start Position: P" << qrCodeParams.startIndex + 1 << "(" << startPosition.x << ", " << startPosition.y <<
                        //    ")\nGoal Position: P" << qrCodeParams.goalIndex + 1 << "(" << goalPosition.x << ", " << goalPosition.y << ")" << "\n";

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
                        crossroadManeuverThreshold = TURNLEFTRIGHTTHRESHOLD/2;
                    }


                    performingTurn = true;
                }
            }

            if (crossroadDetected)
            {                
                turnCounter += timeStep;

                /* turn maneuver should be finished after turn counter has exceeded threshold */                
                if (turnCounter >= crossroadManeuverThreshold)
                {
                    turnCounter = 0;
                    crossroadDetected = false;
                    performingTurn = false;
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
                    /* find an alternative path to circumnavigate detected obstacle */
                    pathplanner->findAlternativePath();

                    turnCounter = 0;
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
