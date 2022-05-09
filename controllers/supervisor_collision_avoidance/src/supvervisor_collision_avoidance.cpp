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

    /*
    *   how to:
    *   communication between one supvervisor entity and multiple robots.
    *   communication medium is wifi. Supervisor opens up a server on a specific port.
    *   Robots try to connect to the server.
    *   When setting up a socket server, the result is a SFD value. 
    *   The clients need this SFD value in order to connect to the correct port.
    *   -> how to communicate this SFD value from supervisor entity to all clients ?
    *   -> using emitter/receiver mechanism by webots as seen in webots' soccer example?
    */
    

    while (robot->step(mTimeStep) != -1) {
        //socket_send(clientSocket, sendbuf, sendbuflen);
        //printf("Sent data %d...\n", socket_send(clientSocket, sendbuf, sendbuflen));

        //emitter->send(&mSocket, sizeof(mSocket));

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