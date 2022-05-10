#include "supvervisor_collision_avoidance.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    emitter = robot->getEmitter("emitter");
    socketServer = new CommModuleTCPSocketServer(1000); // initialise socket server on port 1000

    /*************************************/
    mTimeStep =(unsigned int)robot->getBasicTimeStep();

    //const int DEFAULT_BUFLEN = 512;
    //char sendbuf[DEFAULT_BUFLEN];
    //int sendbuflen = DEFAULT_BUFLEN;

    //sendbuf[0] = '4';
    //sendbuf[1] = '2';
    //sendbuf[2] = '\0';
        

    while (robot->step(mTimeStep) != -1) {

        /* get received data from tcp server via polling */
        /*
        *   data 
        * 
        *   socketServer->getData()
        *   --> interpret data in supervisor? 
        */

    
        //robotActiveWait(1000);
    }

    /*  optional cleanup  */
    delete robot;

    return 0;
}

void robotActiveWait(int numOfSteps)
{
    for (int i = 0; i < numOfSteps; i++) {
        if (robot->step(mTimeStep) == -1)
            break;
    }
}