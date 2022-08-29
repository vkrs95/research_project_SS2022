#include "PathPlannerEPuck.h"
#include "..\include\PathPlannerEPuck.h"

PathPlannerEPuck::PathPlannerEPuck(std::string robotsName)
{
    this->robotName = robotsName;
}

void PathPlannerEPuck::setPath(std::vector<std::tuple<int, int>> path)
{
    /* reset parameters */
    pathCoordinatesList.clear();
    pathIterator = 1;               // initiate internal iterator, must be 1 to have predecessor at index 0 

    pathCoordinatesList = translateToNodeList(path);

    this->startPosition = pathCoordinatesList.at(0);
    this->goalPosition = pathCoordinatesList.at(pathCoordinatesList.size()-1);

    // debug output of start/goal information
    std::cout << "------------------------" << "\n";
    std::cout << "E-Puck '" << robotName << "\n";
    std::cout << "Start Position: " << "(" << startPosition.x_ << ", " << startPosition.y_ <<
        ")\nGoal Position: " << "(" << goalPosition.x_ << ", " << goalPosition.y_ << ")" << "\n";
    std::cout << "------------------------" << "\n";
}

bool PathPlannerEPuck::setAlternativePath(std::vector<std::tuple<int, int>> path)
{
    /* 
    *   check if alternative path contains a loop :
    *   when alternative path is calculated and this robot has right of way 
    *   the same shortest path is set as before but with the collision point 
    *   added in the beginning of the path. 
    *   If the collision point is not included, 
    * 
    *   check if oldPath = newPath except first and second entry
    */

    if (pathIsDifferent(path)) {
        setPath(path);
        return true;
    }

    return false;
}

bool PathPlannerEPuck::pathIsDifferent(std::vector<std::tuple<int, int>> path)
{
    std::vector<Node> newPath = translateToNodeList(path);

    /* debug print */
    //std::cout << "E-Puck '" << robotName << std::endl;
    //for (Node n : newPath) {
    //    std::cout << "x: " << n.x_ << ", y: " << n.y_ << std::endl;
    //}
    //std::cout << "Iterator at " << pathIterator << std::endl;
    //for (Node n : pathCoordinatesList) {
    //    std::cout << "x: " << n.x_ << ", y: " << n.y_ << std::endl;
    //}

    size_t newIterator = pathIterator - 1;
    for (int i = 0; i < newPath.size(); i++) {
        if (newPath.at(i).x_ != pathCoordinatesList.at(newIterator + i).x_ ||
            newPath.at(i).y_ != pathCoordinatesList.at(newIterator + i).y_)
            return true;
    }

    return false;
}


bool PathPlannerEPuck::pathCompleted(void)
{
    /*
    *   coordinate list must have been initialized and iterator has 
    *   processed pathCoordinatesList completely
    */
    return pathCoordinatesList.size() > 0 && pathIterator >= pathCoordinatesList.size();
}

void PathPlannerEPuck::getObstacleParameters(std::tuple<int, int>* startCoords, std::tuple<int, int>* goalCoords, std::tuple<int, int>* collisionCoords, int numberOfSteps)
{
    /*
    *   To circumnavigate the obstacle the robot turns around and
    *   sets the position of the predecessor as its new start position. After this
    *   a new path is planned with the updated nodes and obstacles.
    */
    
    //std::cout << robotName << " steps: " << numberOfSteps << std::endl;

    if (numberOfSteps > STEPS_TO_CROSSROAD_THRESHOLD) {

        startPosition = pathCoordinatesList[pathIterator - 1];

        // add position of current successor as new detected obstacle
        *collisionCoords = std::make_tuple((pathCoordinatesList[pathIterator]).x_,
            (pathCoordinatesList[pathIterator]).y_);
    }
    else if (numberOfSteps > STEPS_TO_INTERMEDIATE_THRESHOLD){

        startPosition = pathCoordinatesList[pathIterator - 2];

        // add position of node before current successor as new detected obstacle
        *collisionCoords = std::make_tuple((pathCoordinatesList[pathIterator - 1]).x_,
            (pathCoordinatesList[pathIterator - 1]).y_);
    }
    else {
        startPosition = pathCoordinatesList[pathIterator - 3];

        // add position of node before current successor as new detected obstacle
        *collisionCoords = std::make_tuple((pathCoordinatesList[pathIterator - 2]).x_,
            (pathCoordinatesList[pathIterator - 2]).y_);
    }    

    *startCoords = std::make_tuple(startPosition.x_, startPosition.y_);

    *goalCoords = std::make_tuple(goalPosition.x_, goalPosition.y_);
}

std::vector<Node> PathPlannerEPuck::translateToNodeList(std::vector<std::tuple<int, int>> tupleList)
{
    std::vector<Node> nodeList;

    for (auto entry : tupleList) {
        Node n(std::get<0>(entry), std::get<1>(entry));
        nodeList.push_back(n);
    }

    return nodeList;
}

MovingDirection PathPlannerEPuck::getNextMovingDirection(void)
{
    MovingDirection nextDirection;

    Node predecessor = pathCoordinatesList.at(pathIterator - 1);
    Node current = pathCoordinatesList.at(pathIterator);
    Node successor = pathCoordinatesList.at(pathIterator + 1);

    predecessor.x_ = predecessor.x_ - current.x_;
    predecessor.y_ = predecessor.y_ - current.y_;

    successor.x_ = successor.x_ - current.x_;
    successor.y_ = successor.y_ - current.y_;

    int sumX = predecessor.x_ + successor.x_;
    int sumY = predecessor.y_ + successor.y_;

    if (predecessor == successor)
    {
        nextDirection = MovingDirection(turnAround);
    }
    else if (sumX == 0 && sumY == 0)
    {
        nextDirection = MovingDirection(straightOn);
    }
    else
    {
        int vectorSumXY = static_cast<int>( (sumX + sumY) * 0.5 );
        int predSucSum = predecessor.y_ + successor.x_ + successor.y_;

        if (vectorSumXY - predSucSum == 0)
        {
            nextDirection = MovingDirection(turnLeft);
        }
        else
        {
            nextDirection = MovingDirection(turnRight);
        }
    }

    /* update iterator, add 2 because of padding coordinates */
    pathIterator += 2;

    return nextDirection;
}