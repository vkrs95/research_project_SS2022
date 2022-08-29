#include "CollisionEvent.h"

/**********************************************************************
* 
*   CollisionEvent class functions 
* 
***********************************************************************/


/*   
*   Objects of this class are used to generate and resolve collision events
* 
*/

CollisionEvent::CollisionEvent(coordinate collisionPoint, PathPlanner* planner)
{
    mCollisionPoint = collisionPoint;
    mParticipants.clear();

    this->planner = planner;

    /* create and run event thread */
    resolveEventThread = new std::thread(&CollisionEvent::resolveEventThreadRoutine, this);
    resolveEventThread->detach();
    mState = EventState::IN_PROGRESS;
}


void CollisionEvent::resolveEventThreadRoutine(void)
{
    /*
    *   SIMPLE COLLISION DISSOLUTION:
    *   calculate distance to goal (dtg) for each robot. 
    *   Robot A with shortest distance to goal has right of way, 
    *   robot B calculates alternative path
    */
    unsigned int timeoutCounter = 0;
    /* for now we expect two participants registered in a collision event */
    while (mParticipants.size() < 2) {
        /* sleep for 100ms and check number of participants again */
        std::this_thread::sleep_for(std::chrono::milliseconds(PARTICIPANT_WAIT_TIME));

        if ((timeoutCounter += PARTICIPANT_WAIT_TIME) >= PARTICIPANT_TIMEOUT)
            break;
    }

    /*
    *   timeout case: collision event has been aborted since only one particpant is registered 
    */
    if (mParticipants.size() < 2) {

        CollisionParticipant* prtcpnt = &mParticipants.at(0);
        /* just set shortest path for participant without collision node included */
        prtcpnt->setPath(planner->getShortestPath(prtcpnt->mStart, prtcpnt->mGoal));

        mState = EventState::CANCELED;
    }
    /*
    *   event handling case: go through all participants and try to resolve collision properly
    */
    else {
        /* find smallest distance to goal out of all robot paths */
        int curDTG = INT16_MAX;
        int minDTG = INT16_MAX;
        int minDTGIndex = -1;

        for (int i = 0; i < mParticipants.size(); i++) {

            curDTG = determineDTG(&mParticipants[i]);

            if (curDTG < minDTG) {
                minDTG = curDTG;
                minDTGIndex = i;
            }
        }

        /* calculate best path for each robot to dissolve collision event */
        for (int i = 0; i < mParticipants.size(); i++) {
            CollisionParticipant* prtcpnt = &mParticipants.at(i);
            if (i == minDTGIndex) {
                /* right of way: continue shortest path */
                prtcpnt->setPath(
                    planner->getShortestPath(prtcpnt->mStart, prtcpnt->mGoal, mCollisionPoint));
            }
            else {
                /* calculate alternative path to circumnavigate collision */
                prtcpnt->setPath(
                    planner->getAlternativePath(prtcpnt->mStart, prtcpnt->mGoal, mCollisionPoint));
            }
        }

        mState = EventState::RESOLVED;
    }
}

int CollisionEvent::determineDTG(CollisionParticipant* participant)
{
    return planner->getShortestPath(participant->mStart, participant->mGoal, mCollisionPoint).size();
}

EventState CollisionEvent::getState(void)
{
    return mState;
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
    return (mState != EventState::UNINITIATED && mState != EventState::IN_PROGRESS);
}


/*
*   Returns a map of all participants with their calculated collision avoidance paths
* 
*   The internal list of participants has to be converted to a map since the 
*   class CollisionParticipants is a protected nested class and for internal 
*   use only. 
*/
std::map<std::string, std::vector<Node>> CollisionEvent::getParticipants(void)
{
    std::map<std::string, std::vector<Node>> participantNamePathMap;

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

std::vector<Node> CollisionEvent::CollisionParticipant::getPath(void)
{
    return path;
}

void CollisionEvent::CollisionParticipant::setPath(std::vector<Node> path)
{
    this->path = path;
}
