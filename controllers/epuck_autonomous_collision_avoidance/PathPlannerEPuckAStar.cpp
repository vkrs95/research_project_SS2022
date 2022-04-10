#include "PathPlannerEPuckAStar.h"

PathPlannerEPuckAStar::PathPlannerEPuckAStar()
{
    alternativePlanningActive = false;
}

void PathPlannerEPuckAStar::findPath(AStar::Vec2i startPosition, AStar::Vec2i goalPosition)
{
    AStar::Generator* generator = new AStar::Generator;

    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1 });
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    addWallsToWorldGenerator(generator);

    coordinateList = generator->findPath(goalPosition, startPosition);

    //// debug print
    //for (int i = 0; i < coordinateList.size(); i++) {
    //    std::cout << "(" << coordinateList.at(i).x << ", " << coordinateList.at(i).y << "); ";
    //}
    //std::cout << "\n";
}

void PathPlannerEPuckAStar::findPath(AStar::CoordinateList newWallsList, AStar::Vec2i startPosition, AStar::Vec2i goalPosition)
{
    coordinateList.clear();
    AStar::Generator* generator = new AStar::Generator;

    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1});
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    for (int i = 0; i < newWallsList.size(); i++)
    {
        generator->addCollision(newWallsList.at(i));
    }

    addWallsToWorldGenerator(generator); // add default walls

    coordinateList = generator->findPath(goalPosition, startPosition);
}



void PathPlannerEPuckAStar::generatePointCoordinateList()
{
    int i;

    // top point row
    for (i = 1; i < MATRIX_N; i += 2) {
        MPList.push_back({ i, 0 });
    }

    // bottom point row
    for (i = 1; i < MATRIX_N; i += 2) {
        MPList.push_back({ i, MATRIX_N });
    }

    // left side points
    for (i = 1; i < MATRIX_N; i += 2) {
        MPList.push_back({ 0, i });
    }

    // right side points
    for (i = 1; i < MATRIX_N; i += 2) {
        MPList.push_back({ MATRIX_N, i });
    }

}

void PathPlannerEPuckAStar::findAlternativePath(AStar::Vec2i newWall, AStar::Vec2i startPosition, AStar::Vec2i goalPosition, RobotHeading currentHeading)
{
    AStar::Generator* generator = new AStar::Generator;

    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1 });
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    addWallsToWorldGenerator(generator);
    generator->addCollision(newWall);

    alternativeHeading = inverseHeading(currentHeading);
    alternativePlanningActive = true;

    coordinateList.clear();
    coordinateList = generator->findPath(goalPosition, startPosition);
    setInstuctionList(coordinateList);
}


void PathPlannerEPuckAStar::setInstuctionList(AStar::CoordinateList path)
{
    int diffX, diffY;
    MovingDirection nextDirection;
    RobotHeading nextHeading;
    RobotHeading epuckCurrentHeading;

    if (alternativePlanningActive)
    {
        epuckCurrentHeading = alternativeHeading;
        alternativePlanningActive = false;

        headingList.clear();
        pathDirectionList.clear();
    }
    else
        epuckCurrentHeading = determineEpuckInitHeading(path.at(0));


    auto firstPos = path.at(1);
    unsigned int lastPosX = firstPos.x;
    unsigned int lastPosY = firstPos.y;

    for (int i = 2; i < path.size(); i++) {
        diffX = path.at(i).x - lastPosX;
        diffY = path.at(i).y - lastPosY;

        if (diffX != 0) 
        {
            switch (epuckCurrentHeading)
            {
            case HEADING_NORTH:
                nextDirection = diffX == 1 ? turnRight : turnLeft;
                nextHeading = HEADING_NORTH;
                break;
            case HEADING_EAST:
                nextDirection = diffX == 1 ? straightOn : turnAround;
                nextHeading = HEADING_EAST;
                break;
            case HEADING_SOUTH:
                nextDirection = diffX == 1 ? turnLeft : turnRight;
                nextHeading = HEADING_SOUTH;
                break;
            case HEADING_WEST:
                nextDirection = diffX == 1 ? turnAround : straightOn;
                nextHeading = HEADING_WEST;
                break;
            default:
                std::cout << "Couldnt determine Heading " << "\n";
                throw std::logic_error("Programm should not reach this state: unknown RobotHeading state reached");
                break;
            }
            epuckCurrentHeading = diffX == 1 ? HEADING_EAST : HEADING_WEST;
        }
                
        else if (diffY != 0) 
        {
            switch (epuckCurrentHeading)
            {
            case HEADING_NORTH:
                nextDirection = diffY == 1 ? turnAround : straightOn;
                nextHeading = HEADING_NORTH;
                break;
            case HEADING_EAST:
                nextDirection = diffY == 1 ? turnRight : turnLeft;
                nextHeading = HEADING_EAST;
                break;
            case HEADING_SOUTH:
                nextDirection = diffY == 1 ? straightOn : turnAround;
                nextHeading = HEADING_SOUTH;
                break;
            case HEADING_WEST:
                nextDirection = diffY == 1 ? turnLeft : turnRight;
                nextHeading = HEADING_WEST;
                break;
            default:
                std::cout << "Couldnt determine Heading " << "\n";
                throw std::logic_error("Programm should not reach this state: unknown RobotHeading state reached");
                break;
            }
            epuckCurrentHeading = diffY == 1 ? HEADING_SOUTH : HEADING_NORTH;
        }

        else {
            throw std::logic_error("Programm should not reach this state: neither x nor y has changed");
        }

        if (nextDirection == turnAround)
            printf(">>>> Next direction shall be turnAround.");

        if (lastPosX % 2 != 0 && lastPosY % 2 != 0) 
        {
            pathDirectionList.push_back(nextDirection);
            headingList.push_back(nextHeading);
        }

        lastPosX = path.at(i).x;
        lastPosY = path.at(i).y;
    }
}


RobotHeading PathPlannerEPuckAStar::determineEpuckInitHeading(AStar::Vec2i currentPos)
{
    RobotHeading epuckHeading;
    unsigned int xPos = currentPos.x;
    unsigned int yPos = currentPos.y;

    if (yPos == 0) {
        epuckHeading = HEADING_SOUTH;
    }
    else if (yPos == MATRIX_N) {
        epuckHeading = HEADING_NORTH;
    }
    else if (xPos == 0) {
        epuckHeading = HEADING_EAST;
    }
    else {
        epuckHeading = HEADING_WEST;
    }

    return epuckHeading;
}

RobotHeading PathPlannerEPuckAStar::inverseHeading(RobotHeading currentHeading)
{
    RobotHeading epuckHeading;

    if (currentHeading == HEADING_NORTH) 
    {
        epuckHeading = HEADING_SOUTH;
    }
    else if (currentHeading == HEADING_SOUTH)
    {
        epuckHeading = HEADING_NORTH;
    }
    else if (currentHeading == HEADING_WEST)
    {
        epuckHeading = HEADING_EAST;
    }
    else if (currentHeading == HEADING_EAST)
    {
        epuckHeading = HEADING_WEST;
    }

    return epuckHeading;
}


void PathPlannerEPuckAStar::addWallsToWorldGenerator(AStar::Generator* generator)
{
    int i, j;

    for (i = 0; i < MATRIX_N + 1; i++) {
        if (i % 2 == 0) {
            for (j = 0; j < MATRIX_N + 1; j++) {
                if (j % 2 == 0) {
                    generator->addCollision({ j,i });
                }
            }
        }
    }
}

void PathPlannerEPuckAStar::setMatrixDimension(unsigned int dimension)
{
    ARENA_NUMBER_OF_LINES_PER_SIDE = dimension;
    MATRIX_N = ARENA_NUMBER_OF_LINES_PER_SIDE * 2;

    // dynamically initialize list of coordinates 
    generatePointCoordinateList();
}


MovingDirection PathPlannerEPuckAStar::getNextMovingDirection(size_t pathIterator)// AStar::Vec2i predecessor, AStar::Vec2i current, AStar::Vec2i successor)
{
    MovingDirection nextDirection;

    AStar::Vec2i predecessor = coordinateList.at(pathIterator - 1);
    AStar::Vec2i current = coordinateList.at(pathIterator);
    AStar::Vec2i successor = coordinateList.at(pathIterator + 1);

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
        int vectorSumXY = (sumX + sumY) * 0.5;
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

    /* debug output for node coordinates */
    //std::cout << "Pathplanning in iteration:" << "( " << pathIterator << " )\n";
    //std::cout << "New direction (0 = Straight, 1 = left, 2 = right)" << "(" << nextDirection << ")\n";
    //std::cout << "Predecessor P" << "(" << predecessor.x << ", " << predecessor.y << ")\n";
    //std::cout << "Current P" << "(" << current.x << ", " << current.y << ")\n";
    //std::cout << "Successor P" << "(" << successor.x << ", " << successor.y << ")\n";
    //std::cout << "---------------------------------------\n";

    return nextDirection;
}