#include "PathPlanningModule.h"
#include "a_star/AStar.hpp"

#include <iostream>
#pragma once

enum MovingDirection {
    straightOn = 0,
    turnLeft,
    turnRight,
    turnAround
};

enum RobotHeading {
    HEADING_NORTH = 0,
    HEADING_EAST,
    HEADING_SOUTH,
    HEADING_WEST
};

class PathPlannerEPuckAStar: 
    public PathPlanningModule<AStar::Vec2i, void>
{
public:
    std::vector<MovingDirection> pathDirectionList;
    std::vector<RobotHeading> headingList;
    AStar::CoordinateList coordinateList;

    RobotHeading alternativeHeading;
    bool alternativePlanningActive;

    PathPlannerEPuckAStar();

    /* overriding abstract parent function */
    void findPath(AStar::Vec2i startPosition, AStar::Vec2i goalPosition) override;

    /* provide additional public functions to use this planner */
    void findPath(AStar::CoordinateList newWallsList, AStar::Vec2i startPosition, AStar::Vec2i goalPosition);
    void findAlternativePath(AStar::Vec2i newWall, AStar::Vec2i startPosition, AStar::Vec2i goalPosition, RobotHeading currentHeading);
    void generatePointCoordinateList();
    void setMatrixDimension(unsigned int dimension);
    MovingDirection getNextMovingDirection(size_t pathIterator);

    std::vector<AStar::Vec2i> MPList = {};

private:
    void setInstuctionList(AStar::CoordinateList path);
    RobotHeading determineEpuckInitHeading(AStar::Vec2i currentPos);
    RobotHeading inverseHeading(RobotHeading currentHeading);
    void addWallsToWorldGenerator(AStar::Generator* generator);

    int ARENA_NUMBER_OF_LINES_PER_SIDE;
    int MATRIX_N;
};
