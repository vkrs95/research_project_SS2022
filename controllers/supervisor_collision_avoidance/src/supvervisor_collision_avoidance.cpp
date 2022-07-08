#include "supvervisor_collision_avoidance.h"

int main(int argc, char **argv) {

    /*** create all Object instances ***/
    robot = new Robot();
    emitter = robot->getEmitter("emitter");
    //camera = robot->getCamera("camera");
    socketServer = new CommModuleTCPSocketServer(1000); // initialise socket server on port 1000

    /*************************************/
    mTimeStep =(unsigned int)robot->getBasicTimeStep();

    std::list<CommModuleTCPSocketServer::PathNotification> pathNotList;
    std::list<CommModuleTCPSocketServer::CollisionNotification> collisionNotList;
    std::map<std::string, std::vector<coordinate>> clientPaths;
    
    /*
    *   Supervisor needs to determine world dimension by 
    *   image processing of scanned line grid camera image 
    */
    // camera->enable(50);
    // camera->saveImage("supervisor_cam_test.jpg", 100);
    // mWorldDimension = getWorldDimensionFromScan("supervisor_cam_test.jpg");

    /*
    *   initialize rest of handler objects
    */
    pathPlanner = new PathPlanner(mWorldDimension); // TODO: replace temporary value with read out dimension
    collisionHandler = new CollisionHandler(pathPlanner);

    /*
    *   supervisor controller main routine
    *   periodically check for various conditions and handle them
    */
    while (robot->step(mTimeStep) != -1) {

        /* check for notifications */
        if (socketServer->checkClientNotifications(&pathNotList, &collisionNotList)) {

            /* check for path requests */
            if (!pathNotList.empty()) {
                /*
                *   some clients have send path messages; calculate paths for each request 
                *   and send them back
                */
                for (CommModuleTCPSocketServer::PathNotification pmsg : pathNotList) {
                    std::vector<coordinate> path = toCoordinateVector(pathPlanner->getShortestPath(pmsg.startNode, pmsg.goalNode));
                    socketServer->sendPathMessage(pmsg.clientName, path);
                }

                pathNotList.clear();
            }

            /* check for collisions */
            if (!collisionNotList.empty()) {
                /*
                *   some clients have send collision messages; reigster them
                *   at collision handler
                */
                for (CommModuleTCPSocketServer::CollisionNotification cmsg : collisionNotList) {

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
        }

        /* check for resolved collisions */
        if (collisionHandler->collisionResolved(&clientPaths)) {
            socketServer->sendCollisionMessageReply(clientPaths);
            clientPaths.clear();
        }

        /* check for */
         
        /* sleep for 50 timesteps */
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

std::vector<coordinate> toCoordinateVector(std::vector <Node> nodeVector)
{
    std::vector<coordinate> coordV;
    for (const auto node : nodeVector) {

        coordV.push_back(std::make_tuple(node.x_, node.y_));
    }

    return coordV;
}