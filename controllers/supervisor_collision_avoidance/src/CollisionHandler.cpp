#include "CollisionHandler.h"

/**********************************************************************
* 
*   CollisionHandler class functions 
* 
***********************************************************************/


/*   
*   Objects of this class are used to ...
* 
*/

CollisionHandler::CollisionHandler(void)
{
}

void CollisionHandler::registerCollision(std::string name, coordinate start, coordinate goal, coordinate collision)
{
    if (mCollisionList[collision] == nullptr) {
        mCollisionList[collision] = new CollisionEvent(collision);
    }

    mCollisionList[collision]->addParticipant(name, start, goal);
}

void CollisionHandler::processCollisionEvents(void)
{
}

bool CollisionHandler::collisionResolved(std::map<std::string, std::vector<coordinate>>* clientPathList)
{

    /*
    *   internally Node is used for a path step, but we are exposing tuple type coordinate
    *   thus we have to convert Node to coordinate type
    */
    std::map<std::string, std::vector<coordinate>>& clientPathListRef = *clientPathList;
    
    /* go through all collision events and check their event resolved state */
    for (const auto collEvent : mCollisionList) {

        if (collEvent.second->eventResolved()) {

            std::cout << "CollisionHandler: Collision resolved at " << std::get<0>(collEvent.first) << "," << std::get<1>(collEvent.first) << std::endl;
            std::map<std::string, std::vector<Node>> resolvedParticipants = collEvent.second->getParticipants();
            
            /* go through all participants of an event */
            for (const auto participant : resolvedParticipants) {

                /* go through all nodes of a path and add them to the coordinate vector */
                for (auto pathNode: participant.second) {
                    clientPathListRef[participant.first].push_back(std::make_tuple(pathNode.x_, pathNode.y_));
                }
            }

            /* event is resolved, remove and delete object from map */
            mCollisionList.erase(collEvent.first);
        }
    }

    return clientPathList->size() > 0;
}
