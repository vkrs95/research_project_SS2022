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
    RobotHeading alternativeHeading;
    bool alternativePlanningActive;

    /* constructor */
    PathPlannerEPuckAStar();

    /*
    *   planner function to find path between a given start and goal position.
    *   if no arguments are passed, use start and goal variables that have been 
    *   configured beforehand.
    */
    void findPath(AStar::Vec2i startPosition = {}, AStar::Vec2i goalPosition = {}) override;

    /* provide additional public functions to use this planner */
    void findAlternativePath(void);
    bool pathCompleted(void);
    void setMatrixDimension(unsigned int dimension);
    void setStartGoalPositionByIndex(unsigned int startIndex, unsigned int goalIndex);

    MovingDirection getNextMovingDirection(void);

private:
    void generateEdgeNodeList();
    void setInstuctionList(AStar::CoordinateList path);
    RobotHeading determineEpuckInitHeading(AStar::Vec2i currentPos);
    RobotHeading inverseHeading(RobotHeading currentHeading);
    void addWallsToWorldGenerator(AStar::Generator* generator);

    /*
    *   List of all edge nodes as 2D coordinates.
    *   Nodes are saved in the following order: top, bottom, left, right.
    *   The list must be generated beforehand in order to calculate a path
    *   between a start and goal edge node.
    */
    std::vector<AStar::Vec2i> edgeNodeList = {};

    /* list of (x,y) coordinates from start to goal position */
    AStar::CoordinateList pathCoordinatesList;
    
    /* internal iterator to work through pathCoordinatesList step by step */
    size_t pathIterator;

    /* current start and goal of the robot */
    AStar::Vec2i startPosition, goalPosition;

    /* list of detected obstacles */
    AStar::CoordinateList obstacleList;

    /* constants */
    int ARENA_NUMBER_OF_LINES_PER_SIDE;
    int MATRIX_N;
};
