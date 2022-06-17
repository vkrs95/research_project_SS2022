#include "PathPlannerEPuck.h"
#include "..\include\PathPlannerEPuck.h"

PathPlannerEPuck::PathPlannerEPuck(std::string robotsName)
{
    this->robotName = robotsName;
}

void PathPlannerEPuck::prepareGridAndRunPlanner(bool addObstaclesFromList)
{

    /* initate world grid */
    prepareWorldGrid(addObstaclesFromList);
    //AStar aStarPlanning(worldGrid);
    Dijkstra dijkstraPlanning(worldGrid);

    /* calculate path between start and goal */
    //auto [planningSuccessful, pathVector] = aStarPlanning.Plan(this->startPosition, this->goalPosition);
    auto [planningSuccessful, pathVector] = dijkstraPlanning.Plan(this->startPosition, this->goalPosition);

    /* assign results to member variables */
    lastPathPlanningSuccessful = planningSuccessful;
    pathCoordinatesList = pathVector;

    /* add obstacle as entry to determine direction on first crossroad */
    if (addObstaclesFromList) {
        pathCoordinatesList.push_back(obstacleList.at(0));
    }

    /*
    *   pathCoordinatesList is ordered goal to start,
    *   reverse list to have steps from start to goal instead
    */
    std::reverse(pathCoordinatesList.begin(), pathCoordinatesList.end());
}

void PathPlannerEPuck::findPath(Node startPosition, Node goalPosition)
{
    if (startPosition.x_ == 0 && startPosition.y_ == 0 &&
        goalPosition.x_ == 0 && goalPosition.y_ == 0) {
        /*
        *   no parameters passed, use preconfigured start and goal position 
        *   --> nothing to do here
        */ 
    } 
    else {
        /* save passed positions in internal variables */
        this->startPosition = startPosition;
        this->goalPosition = goalPosition;
    }

    /* initate world grid and calculate path between start and goal */    
    prepareGridAndRunPlanner();
    
    /* first time path planning, initiate list of obstacles */
    obstacleList.clear();
}

void PathPlannerEPuck::getObstacleParameters(std::tuple<int, int> *startCoords, std::tuple<int, int> *goalCoords, std::tuple<int, int> *collisionCoords)
{
    /*
    *   To circumnavigate the obstacle the robot turns around and
    *   sets the position of the predecessor as its new start position. After this
    *   a new path is planned with the updated nodes and obstacles.
    */
    startPosition = pathCoordinatesList[pathIterator - 2];

    *startCoords = std::make_tuple(startPosition.x_, startPosition.y_);

    *goalCoords = std::make_tuple(goalPosition.x_, goalPosition.y_);
    
    // add position of current successor as new detected obstacle
    *collisionCoords = std::make_tuple((pathCoordinatesList[pathIterator - 1]).x_,
                        (pathCoordinatesList[pathIterator - 1]).y_ );    
}

void PathPlannerEPuck::runAlternativePath(std::vector<std::tuple<int, int>> altPath)
{
    /* reset parameters */
    pathCoordinatesList.clear();
    pathIterator = 1;               // initiate internal iterator, must be 1 to have predecessor at index 0 

    lastPathPlanningSuccessful = true;
    pathCoordinatesList = translateToNodeList(altPath);
}

bool PathPlannerEPuck::pathCompleted(void)
{
    /*
    *   coordinate list must have been initialized and iterator has 
    *   processed pathCoordinatesList completely
    */
    return pathCoordinatesList.size() > 0 && pathIterator >= pathCoordinatesList.size();
}

void PathPlannerEPuck::generateEdgeNodeList()
{
    int i;

    // top point row
    for (i = 1; i < MATRIX_N; i += 2) {
        edgeNodeList.push_back({ i, 0 });
    }

    // bottom point row
    for (i = 1; i < MATRIX_N; i += 2) {
        edgeNodeList.push_back({ i, MATRIX_N });
    }

    // left side points
    for (i = 1; i < MATRIX_N; i += 2) {
        edgeNodeList.push_back({ 0, i });
    }

    // right side points
    for (i = 1; i < MATRIX_N; i += 2) {
        edgeNodeList.push_back({ MATRIX_N, i });
    }

}

void PathPlannerEPuck::prepareWorldGrid(bool addObstaclesFromList)
{
    int n = MATRIX_N + 1;

    /* reset planner parameters*/
    pathCoordinatesList.clear();
    pathIterator = 1;               // initiate internal iterator, must be 1 to have predecessor at index 0 
    worldGrid.resize(n, std::vector<int>(n, 0));

    /* add default walls as collision points to world grid */
    for (int i = 0; i < n; i++) {
        if (i % 2 == 0) {
            for (int j = 0; j < n; j++) {
                if (j % 2 == 0) {
                    worldGrid[j][i] = 1;            // add wall
                }
            }
        }
    }

    /* if flag is set, add obstacles from list as additional collision points to world generator */
    if (addObstaclesFromList) {
        for (int i = 0; i < obstacleList.size(); i++)
        {
            Node obNode = obstacleList.at(i);
            worldGrid[obNode.x_][obNode.y_] = 1;
        }
    }

    startPosition.id_ = startPosition.x_ * n + startPosition.y_;
    startPosition.pid_ = startPosition.x_ * n + startPosition.y_;
    goalPosition.id_ = goalPosition.x_ * n + goalPosition.y_;
    // use static cast to avoid overflow warning when casting a 4 byte result to an 8 byte value
    startPosition.h_cost_ = abs(static_cast<double>(startPosition.x_) - goalPosition.x_) + abs(static_cast<double>(startPosition.y_) - goalPosition.y_);
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

void PathPlannerEPuck::setMatrixDimension(unsigned int dimension)
{
    ARENA_NUMBER_OF_LINES_PER_SIDE = dimension;
    MATRIX_N = ARENA_NUMBER_OF_LINES_PER_SIDE * 2;

    // dynamically initialize list of edge node coordinates 
    generateEdgeNodeList();
}

void PathPlannerEPuck::setStartGoalPositionByIndex(unsigned int startIndex, unsigned int goalIndex)
{
    startPosition = edgeNodeList.at(startIndex);
    goalPosition = edgeNodeList.at(goalIndex);

    // debug output of start/goal information
    std::cout << "------------------------" << "\n";
    std::cout << "E-Puck '" << robotName << "' in " << ARENA_NUMBER_OF_LINES_PER_SIDE << "x" << ARENA_NUMBER_OF_LINES_PER_SIDE << " map\n";
    std::cout << "Start Position: P" << startIndex + 1 << "(" << startPosition.x_ << ", " << startPosition.y_ <<
        ")\nGoal Position: P" << goalIndex + 1 << "(" << goalPosition.x_ << ", " << goalPosition.y_ << ")" << "\n";
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