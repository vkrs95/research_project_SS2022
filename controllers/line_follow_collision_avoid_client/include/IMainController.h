
#pragma once

/*
*   This interface represents the main controller module, 
*   its properties and abstract functions
*/
class IMainController
{

public:
    virtual void robotActiveWait(int numOfSteps) = 0;
    virtual int mainControllerRoutine(int argc, char** argv) = 0;
};
