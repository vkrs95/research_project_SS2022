#include "CollisionEvent.h"

/**********************************************************************
* 
*   CollisionHandler class functions 
* 
***********************************************************************/


/*   
*   Objects of this class are used to ...
* 
*/

CollisionEvent::CollisionEvent(coordinate collisionPoint)
{
    mCollisionPoint = collisionPoint;
    mParticipants.clear();
}

void CollisionEvent::addParticipant(CollisionParticipant newParticipant)
{
    mParticipants.push_back(newParticipant);
}

void CollisionEvent::addParticipant(std::string name, coordinate start, coordinate goal)
{
    CollisionParticipant newParticipant(name, start, goal);
    addParticipant(newParticipant);
}

bool CollisionEvent::eventResolved(void)
{
    return mResolved;
}

/*
*   Returns a map of all participants with their calculated collision avoidance paths
* 
*   The internal list of participants has to be converted to a map since the 
*   class CollisionParticipants is a protected nested class and for internal 
*   use only. 
*/
std::map<std::string, std::vector<coordinate>> CollisionEvent::getParticipants(void)
{
    std::map<std::string, std::vector<coordinate>> participantNamePathMap;

    /*  
    *   go through all participants and add their paths to the map 
    *   combined with their unique names as key value
    */
    for (CollisionParticipant p : mParticipants) {
        participantNamePathMap[p.mName] = p.getPath();
    }

    return participantNamePathMap;
}


/******************************************
* 
*   CollisionParticipant class functions
*
******************************************/

CollisionEvent::CollisionParticipant::CollisionParticipant(std::string name, coordinate start, coordinate goal)
{
    mName = name;
    mStart = start;
    mGoal = goal;
}

std::vector<coordinate> CollisionEvent::CollisionParticipant::getPath(void)
{
    return path;
}

void CollisionEvent::CollisionParticipant::setPath(std::vector<coordinate> path)
{
    this->path = path;
}
