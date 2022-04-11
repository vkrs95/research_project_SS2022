#include "PathPlannerEPuckAStar.h"

PathPlannerEPuckAStar::PathPlannerEPuckAStar(std::string robotsName)
{
    this->robotName = robotsName;
}

void PathPlannerEPuckAStar::findPath(AStar::Vec2i startPosition, AStar::Vec2i goalPosition)
{
    if (startPosition.x == 0 && startPosition.y == 0 &&
        goalPosition.x == 0 && goalPosition.y == 0) {
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

    /* initate generator object and calculate path between start and goal */
    AStar::Generator* generator = new AStar::Generator;
    prepareWorldGenerator(generator);
    pathCoordinatesList = generator->findPath(this->goalPosition, this->startPosition);
    
    /* first time path planning, initiate list of obstacles */
    obstacleList.clear();
}

void PathPlannerEPuckAStar::findAlternativePath(void)
{
    // TODO: change ?
    // a robot can only avoid one obstacle at a time thus remove all known ones 
    // before a new obstacle
    obstacleList.clear();

    /* add position of current successor as new detected obstacle */
    obstacleList.push_back(pathCoordinatesList[pathIterator]);
        
    /*
    *   To circumnavigate the obstacle the robot turns around and 
    *   sets the position of the predecessor as its new start position. After this
    *   a new path is planned with the updated nodes and obstacles.
    */
    startPosition = pathCoordinatesList[pathIterator - 1];

    /* initate generator object and calculate path between start and goal */
    AStar::Generator* generator = new AStar::Generator;
    prepareWorldGenerator(generator, true);
    pathCoordinatesList = generator->findPath(goalPosition, startPosition);


    /* debug output of start / goal information */
    unsigned int obstaclePosX = obstacleList.at(obstacleList.size() - 1).x;
    unsigned int obstaclePosY = obstacleList.at(obstacleList.size() - 1).y;

    std::cout << "------------------------" << "\n";
    std::cout << "E-Puck '" << robotName << "' in " << ARENA_NUMBER_OF_LINES_PER_SIDE << "x" << ARENA_NUMBER_OF_LINES_PER_SIDE << " map\n";
    std::cout << "Collision detected at (" << obstaclePosX << ", " << obstaclePosY << ")! Alternative path planning:" << "\n";
    std::cout << "New Start Position: (" << startPosition.x << ", " << startPosition.y <<
        ")\nGoal Position: (" << goalPosition.x << ", " << goalPosition.y << ")" << "\n";
}


bool PathPlannerEPuckAStar::pathCompleted(void)
{
    /*
    *   coordinate list must have been initialized and iterator has 
    *   processed pathCoordinatesList completely
    */
    return pathCoordinatesList.size() > 0 && pathIterator >= pathCoordinatesList.size();
}


void PathPlannerEPuckAStar::generateEdgeNodeList()
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

void PathPlannerEPuckAStar::prepareWorldGenerator(AStar::Generator* generator, bool addObstaclesFromList)
{
    int i, j;

    /* reset planner parameters*/
    pathCoordinatesList.clear();
    pathIterator = 1;               // initiate internal iterator, must be 1 to have predecessor at index 0 

    /* initiate generator and planner parameters*/
    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1 });
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    /* add default walls as collision points to world generator */
    for (i = 0; i < MATRIX_N + 1; i++) {
        if (i % 2 == 0) {
            for (j = 0; j < MATRIX_N + 1; j++) {
                if (j % 2 == 0) {
                    generator->addCollision({ j,i });
                }
            }
        }
    }

    /* if flag is set, add obstacles from list as additional collision points to world generator */
    if (addObstaclesFromList) {
        for (int i = 0; i < obstacleList.size(); i++)
        {
            generator->addCollision(obstacleList.at(i));
        }
    }
}

void PathPlannerEPuckAStar::setMatrixDimension(unsigned int dimension)
{
    ARENA_NUMBER_OF_LINES_PER_SIDE = dimension;
    MATRIX_N = ARENA_NUMBER_OF_LINES_PER_SIDE * 2;

    // dynamically initialize list of edge node coordinates 
    generateEdgeNodeList();
}

void PathPlannerEPuckAStar::setStartGoalPositionByIndex(unsigned int startIndex, unsigned int goalIndex)
{
    startPosition = edgeNodeList.at(startIndex);
    goalPosition = edgeNodeList.at(goalIndex);

    // debug output of start/goal information
    std::cout << "------------------------" << "\n";
    std::cout << "E-Puck '" << robotName << "' in " << ARENA_NUMBER_OF_LINES_PER_SIDE << "x" << ARENA_NUMBER_OF_LINES_PER_SIDE << " map\n";
    std::cout << "Start Position: P" << startIndex + 1 << "(" << startPosition.x << ", " << startPosition.y <<
        ")\nGoal Position: P" << goalIndex + 1 << "(" << goalPosition.x << ", " << goalPosition.y << ")" << "\n";
}


MovingDirection PathPlannerEPuckAStar::getNextMovingDirection(void)
{
    MovingDirection nextDirection;

    AStar::Vec2i predecessor = pathCoordinatesList.at(pathIterator - 1);
    AStar::Vec2i current = pathCoordinatesList.at(pathIterator);
    AStar::Vec2i successor = pathCoordinatesList.at(pathIterator + 1);

    predecessor.x = predecessor.x - current.x;
    predecessor.y = predecessor.y - current.y;

    successor.x = successor.x - current.x;
    successor.y = successor.y - current.y;

    int sumX = predecessor.x + successor.x;
    int sumY = predecessor.y + successor.y;

    if (sumX == 0 && sumY == 0)
    {
        nextDirection = MovingDirection(straightOn);
    }
    else
    {
        int vectorSumXY = static_cast<int>( (sumX + sumY) * 0.5 );
        int predSucSum = predecessor.y + successor.x + successor.y;

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