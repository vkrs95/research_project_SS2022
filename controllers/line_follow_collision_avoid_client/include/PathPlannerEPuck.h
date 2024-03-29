#include "IPathPlanningModule.h"
#include "utils/utils.hpp"

#include <iostream>
#include <tuple>
#pragma once

enum MovingDirection {
    straightOn = 0,
    turnLeft,
    turnRight,
    turnAround
};

class PathPlannerEPuck: 
    public IPathPlanningModule<Node, void>
{
public:

    /*********************************************************
    *
    *	public functions
    *
    *********************************************************/

    /* constructor */
    PathPlannerEPuck(std::string robotName = "E-Puck");

    /* provide additional public functions to use this planner */
    void setPath(std::vector<std::tuple<int, int>> path);
    bool setAlternativePath(std::vector<std::tuple<int, int>> path);
    bool pathCompleted(void);
    void getObstacleParameters(std::tuple<int,int> *startCoords, std::tuple<int, int> *goalCoords, std::tuple<int, int> *collisionCoords, int numberOfSteps);

    MovingDirection getNextMovingDirection(void);

private:

    /*********************************************************
    *
    *	private functions
    *
    *********************************************************/

    std::vector<Node> translateToNodeList(std::vector<std::tuple<int, int>> tupleList);
    bool pathIsDifferent(std::vector<std::tuple<int, int>> path);

    /*********************************************************
    *
    *	internal constants
    *
    *********************************************************/

    const unsigned int STEPS_TO_INTERMEDIATE_THRESHOLD = 60;
    const unsigned int STEPS_TO_CROSSROAD_THRESHOLD = 150;

    /*********************************************************
    *
    *	member variables
    *
    *********************************************************/

    /* list of (x,y) coordinates from start to goal position */
    std::vector<Node> pathCoordinatesList;
    
    /* internal iterator to work through pathCoordinatesList step by step */
    size_t pathIterator;

    /* current start and goal of the robot */
    Node startPosition, goalPosition;

    /* optional name of robot for debug output */
    std::string robotName = "";
};
