#include "CollisionHandler.h"

/**********************************************************************
* 
*   CollisionHandler class functions 
* 
***********************************************************************/


/*   
*   This class represents a collision handler used to manage collision event
*   objects and their participants. 
* 
*/

CollisionHandler::CollisionHandler(PathPlanner* planner)
{
    mPlanner = planner;
}

void CollisionHandler::registerCollision(std::string name, coordinate start, coordinate goal, coordinate collision)
{
    if (mCollisionList[collision] == nullptr) {
        mCollisionList[collision] = new CollisionEvent(collision, mPlanner);
    }

    mCollisionList[collision]->addParticipant(name, start, goal);
}

void CollisionHandler::processCollisionEvents(void)
{
}

bool CollisionHandler::collisionResolved(std::map<std::string, std::pair<int, std::vector<coordinate>>>* clientPathList)
{

    /*
    *   internally Node is used for a path step, but we are exposing tuple type coordinate
    *   thus we have to convert Node to coordinate type
    */
    std::map<std::string, std::pair<int, std::vector<coordinate>>>& clientPathListRef = *clientPathList;
    
    /* go through all collision events and check their event resolved state */
    for (const auto collEvent : mCollisionList) {

        const coordinate collCoord = collEvent.first;
        CollisionEvent* event = collEvent.second;

        if (event->eventResolved()) {

            std::cout << "CollisionHandler: Collision resolved at " << std::get<0>(collCoord) << "," << std::get<1>(collCoord) << std::endl;
            std::map<std::string, std::vector<Node>> resolvedParticipants = event->getParticipants();
            
            /* go through all participants of an event */
            for (const auto prtcpt : resolvedParticipants) {

                const std::string name = prtcpt.first;
                
                /* save state of event */
                clientPathListRef[name].first = static_cast<int>(event->getState());

                /* go through all nodes of a path and add them to the coordinate vector */
                for (auto pathNode: prtcpt.second) {
                    clientPathListRef[name].second.push_back(std::make_tuple(pathNode.x_, pathNode.y_));
                }
            }

            /* event is resolved, remove and delete object from map */
            mCollisionList.erase(collCoord);
        }
    }

    return clientPathList->size() > 0;
}
