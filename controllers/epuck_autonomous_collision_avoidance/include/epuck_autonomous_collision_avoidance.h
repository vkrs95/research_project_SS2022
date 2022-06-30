#include "RobotControlEPuck.h"
#include "PathPlannerEPuck.h"
#include "QRModuleEPuckSGD.h"
#include "CommModuleTCP.h"

#pragma once

/*** object pointers ***/
Robot* robot;
IRobotControl* robotControl;
PathPlannerEPuck* pathplanner;
IQRModule<SGDQRParams>* qrmodule;
ICommModule* commModule;

void robotActiveWait(int numOfSteps);

/*** define member variables ***/
int timeStep;
unsigned int turnCounter = 0;
unsigned int crossroadManeuverThreshold;    // is set when next direction is read  
const unsigned int TURNLEFTRIGHTTHRESHOLD = 3200;
const unsigned int TURNAROUNDTHRESHOLD = 3500; // TURNLEFTRIGHTTHRESHOLD;


/* robot parameters */
unsigned int groundSensorJitter = 0;
unsigned int groundSensorJitterThreshold = 4;
bool endlessMode = true;
unsigned int qrDistanceToScanPos = 2200;
unsigned int initProcedureDistanceToScanCounter = 0;


/* robot routine flags */
bool obstacleDetected = false;
bool crossroadManeuverActive = false;
bool performingTurn = false;
bool endOfLineGoalReached = false;
bool initProcedureDone = false;
bool pathPlanningCompleted = false;
bool supervisorConnected = false;
bool alternativePathReceived = false;
unsigned int readQrCodeAttemptCounter = 0;
unsigned int readQrCodeAttemptLimit = 3;
