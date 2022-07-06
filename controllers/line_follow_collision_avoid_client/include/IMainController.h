
#pragma once

class IMainController
{

public:
    virtual void robotActiveWait(int numOfSteps) = 0;
    virtual int mainControllerRoutine(int argc, char** argv) = 0;
};
