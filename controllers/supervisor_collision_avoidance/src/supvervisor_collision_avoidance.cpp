#include "supvervisor_collision_avoidance.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    emitter = robot->getEmitter("emitter");
    socketServer = new CommModuleTCPSocketServer(1000); // initialise socket server on port 1000
    collisionHandler = new CollisionHandler();

    /*************************************/
    mTimeStep =(unsigned int)robot->getBasicTimeStep();

    std::list<CollisionNotification> collisionNotList;
    std::map<std::string, std::vector<coordinate>> clientPaths;

    while (robot->step(mTimeStep) != -1) {

        /* check for collisions periodically */
        if (socketServer->checkCollisionNotifications(&collisionNotList)) {
            /* 
            *   some clients have send collision messages; reigster them
            *   at collision handler
            */
            for (CollisionNotification cmsg : collisionNotList) {

                std::cout << "Supervisor: received collision at (" 
                    << std::get<0>(cmsg.collisionNode)
                    << ", "
                    << std::get<1>(cmsg.collisionNode)
                    << ") by "
                    << cmsg.clientName
                    << std::endl;

                collisionHandler->registerCollision(cmsg.clientName, cmsg.startNode, cmsg.goalNode, cmsg.collisionNode);
            }

            collisionNotList.clear();
        }

        if (collisionHandler->collisionResolved(&clientPaths)) {
            socketServer->sendCollisionMessageReply(clientPaths);
            clientPaths.clear();
        }
            
        robotActiveWait(50);
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