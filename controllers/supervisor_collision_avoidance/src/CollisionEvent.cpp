#include "CollisionEvent.h"

/**********************************************************************
* 
*   CollisionEvent class functions 
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

    /* create and run event thread */
    resolveEventThread = new std::thread(&CollisionEvent::resolveEventThreadRoutine, this);
    resolveEventThread->detach();
}


void CollisionEvent::resolveEventThreadRoutine(void)
{
    /*
    *   SIMPLE COLLISION DISSOLUTION:
    *   calculate distance to goal (dtg) for each robot. 
    *   Robot A with shortest distance to goal has right of way, robot B
    *   calculates alternative path
    */

    /* for now we expect two participants registered in a collision event */
    while (mParticipants.size() < 2) {
        /* sleep for 100ms and check number of participants again */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    /*** go through all participants and try to resolve collision properly ***/
    
    /* find smallest distance to goal out of all robot paths */
    int curDTG = INT16_MAX;
    int minDTG = INT16_MAX;
    int minDTGIndex = -1;
    
    for(int i = 0; i < mParticipants.size(); i++) {

        // curDTG = determineDTG(mParticipants[i]);
        if (curDTG < minDTG) {
            minDTG = curDTG;
            minDTGIndex = i;
        }
        
    }

    /* calculate best path for each robot to dissolve collision event */
    for (int i = 0; i < mParticipants.size(); i++) {

        if (i == minDTGIndex) {
            /* right of way: continue shortest path */
            // TODO: calculate shortest path to goal and set mParticipants[i].setPath(path)
        }
        else {
            /* calculate alternative path to circumnavigate collision */
            // TODO: set obstacle at collison coordinate, calculate shortest path to goal 
            // and set mParticipants[i].setPath(path)
        }
    }

    mResolved = true;

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