#include "RobotControlEPuck.h"
#include "PathPlannerEPuck.h"
#include "QRModuleEPuckSGD.h"
#include "CommModuleTCP.h"

#pragma once

/* include abstract module interface */
#include "IMainController.h"

class MainControllerEPuck :
    public IMainController
{

public:

    MainControllerEPuck(void);

    /*********************************************************
    *
    *	public functions
    *
    *********************************************************/

    void robotActiveWait(int numOfSteps);
    int mainControllerRoutine(int argc, char** argv);

private:


    /*********************************************************
    *
    *	private functions
    *
    *********************************************************/
    void controlLEDHandling(void);
    int  qrScanHandling(void);
    void initPathHandling(void);
    void crossroadDetectionHandling(void);
    void obstacleHandling(void);
    void goalReachedHandling(void);
    void resetControllerStates(void);

    /*********************************************************
    *
    *	internal constants
    *
    *********************************************************/

    const unsigned int TURN_LEFT_RIGHT_THRESHOLD = 3200;
    const unsigned int TURN_AROUND_THRESHOLD = 3500;
    const unsigned int STRAIGHT_AHEAD_THRESHOLD = 400;
    const unsigned int GROUND_SENSOR_JITTER_THRESHOLD = 4;
    const unsigned int QR_SCAN_DISTANCE_THRESHOLD = 2200;
    const unsigned int QR_READ_ATTEMPTS_THRESHOLD = 3;


    /*********************************************************
    *
    *	member variables
    *
    *********************************************************/

    int timeStep = 0;
    unsigned int crossroadManeuverThreshold = 0;    // is set when next direction is read  
    std::tuple <int, int> mStartNode, mGoalNode;    // local copies of start and goal node that has been read from QR code            
    SGDQRParams qrCodeParams;                       // structure to save read QR parameters into

    /*** object pointers ***/
    Robot* robot;
    IRobotControl* robotControl;
    PathPlannerEPuck* pathplanner;
    IQRModule<SGDQRParams>* qrmodule;
    ICommModule* commModule;

    /* state timestep counters */
    unsigned int turnCounter = 0;
    unsigned int stepCounter = 0;
    unsigned int groundSensorJitter = 0;
    unsigned int readQrCodeAttemptCounter = 0;
    unsigned int initProcedureDistanceToScanCounter = 0;

    /* robot routine flags */
    bool endlessMode = true;
    bool obstacleHandlingActive = false;
    bool crossroadManeuverActive = false;
    bool performingTurn = false;
    bool endOfLineGoalReached = false;
    bool initProcedureDone = false;
    bool pathPlanningCompleted = false;
    bool supervisorConnected = false;
    bool alternativePathReceived = false;
    bool pathRequestSent = false;
    bool qrScanCompleted = false;

};
