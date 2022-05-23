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

bool CollisionHandler::collisionResolved(std::map<std::string, std::vector<Node>>* clientPathList)
{
    /* go through all collision events and check their event resolved state */
    for (const auto collEvent : mCollisionList) {

        if (collEvent.second->eventResolved()) {
            std::map<std::string, std::vector<Node>> resolvedParticipants = collEvent.second->getParticipants();

            /* go through all participants of an event and add them to passed clientPathList */
            for (const auto participant : resolvedParticipants) {
                clientPathList->insert_or_assign(participant.first, participant.second);
            }
        }
    }

    return clientPathList->empty();
}
