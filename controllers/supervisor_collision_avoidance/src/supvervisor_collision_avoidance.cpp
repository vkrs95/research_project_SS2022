#include "supvervisor_collision_avoidance.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    emitter = robot->getEmitter("emitter");
    socketServer = new CommModuleTCPSocketServer(1000); // initialise socket server on port 1000

    /*************************************/
    mTimeStep =(unsigned int)robot->getBasicTimeStep();

    std::list<CollisionNotification> collisionNotList;

    while (robot->step(mTimeStep) != -1) {

        /* check for collisions */
        if (socketServer->checkCollisionNotifications(&collisionNotList)) {
            /* 
            *   some clients have send collision messages; reigster them
            *   at collision handler
            */
            for (CollisionNotification collisionMsg : collisionNotList) {

                //  TODO: implement registration
                //  CollisionHandler.register(name, start, goal, collision)
                std::cout << "Supervisor: received collision at (" 
                    << std::get<0>(collisionMsg.collisionNode) 
                    << ", "
                    << std::get<1>(collisionMsg.collisionNode)
                    << ") by "
                    << collisionMsg.clientName
                    << std::endl;
            }

            collisionNotList.clear();
        }
    
        robotActiveWait(1000);
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