#include "PathPlanningModule.h"
#include "path_planning/a_star.hpp"
#include "path_planning/dijkstra.hpp"

#include <iostream>
#pragma once

enum MovingDirection {
    straightOn = 0,
    turnLeft,
    turnRight,
    turnAround
};

class PathPlannerEPuckAStar: 
    public PathPlanningModule<Node, void>
{
public:
    /* constructor */
    PathPlannerEPuckAStar(std::string robotName = "E-Puck");

    /*
    *   planner function to find path between a given start and goal position.
    *   if no arguments are passed, use start and goal variables that have been 
    *   configured beforehand.
    */
    void findPath(Node startPosition = {}, Node goalPosition = {}) override;

    /* provide additional public functions to use this planner */
    void findAlternativePath(void);
    bool pathCompleted(void);
    void setMatrixDimension(unsigned int dimension);
    void setStartGoalPositionByIndex(unsigned int startIndex, unsigned int goalIndex);

    MovingDirection getNextMovingDirection(void);

private:
    void generateEdgeNodeList();
    void prepareGridAndRunPlanner(bool addObstaclesFromList = false);
    void prepareWorldGrid(bool addObstaclesFromList);

    /*
    *   List of all edge nodes as 2D coordinates.
    *   Nodes are saved in the following order: top, bottom, left, right.
    *   The list must be generated beforehand in order to calculate a path
    *   between a start and goal edge node.
    */
    std::vector<Node> edgeNodeList = {};

    /* list of (x,y) coordinates from start to goal position */
    std::vector<Node> pathCoordinatesList;
    
    /* internal iterator to work through pathCoordinatesList step by step */
    size_t pathIterator;

    /* current start and goal of the robot */
    Node startPosition, goalPosition;

    /* list of detected obstacles */
    std::vector<Node> obstacleList;

    /* constants */
    int ARENA_NUMBER_OF_LINES_PER_SIDE = 0;
    int MATRIX_N = 0;

    /* optional name of robot for debug output */
    std::string robotName = "";

    bool lastPathPlanningSuccessful = false;

    std::vector<std::vector<int>> worldGrid; // TODO: must be initialized in setMatrixDimension() with set ARENA_NUMBER_OF_LINES_PER_SIDE
};
