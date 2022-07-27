#include "PathPlanner.h"

PathPlanner::PathPlanner(int dimension)
{
    worldDimension = dimension;
}

std::vector<Node> PathPlanner::getShortestPath(std::tuple<int, int> start, std::tuple<int, int> goal, std::tuple<int, int> collision)
{
    startPosition.x_ = std::get<0>(start); startPosition.y_ = std::get<1>(start);
    goalPosition.x_ = std::get<0>(goal); goalPosition.y_ = std::get<1>(goal);

    prepareWorldGrid();
        
    return doPathPlanning(collision);
}

std::vector<Node> PathPlanner::getAlternativePath(std::tuple<int, int> start, std::tuple<int, int> goal, std::tuple<int, int> collision)
{
    startPosition.x_ = std::get<0>(start); startPosition.y_ = std::get<1>(start);
    goalPosition.x_ = std::get<0>(goal); goalPosition.y_ = std::get<1>(goal);

    prepareWorldGrid();
    
    /* add obstacle to circumnavigate other robot */
    Node obstacle(std::get<0>(collision), std::get<1>(collision));
    addObstacle(obstacle);

    return doPathPlanning(collision);
}


void PathPlanner::prepareWorldGrid(void)
{
    int n = worldDimension*2 + 1; // arena lines per side * 2 + 1 for walls between each line  

    /* reset world grid */
    worldGrid.clear();
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
        
    startPosition.id_ = startPosition.x_ * n + startPosition.y_;
    startPosition.pid_ = startPosition.x_ * n + startPosition.y_;
    goalPosition.id_ = goalPosition.x_ * n + goalPosition.y_;
    // use static cast to avoid overflow warning when casting a 4 byte result to an 8 byte value
    startPosition.h_cost_ = abs(static_cast<double>(startPosition.x_) - goalPosition.x_) + abs(static_cast<double>(startPosition.y_) - goalPosition.y_);
}

void PathPlanner::addObstacle(Node obstacle)
{
    worldGrid[obstacle.x_][obstacle.y_] = 1;
}

std::vector<Node> PathPlanner::doPathPlanning(std::tuple<int, int> collision)
{
    Dijkstra dijkstraPlanning(worldGrid);

    /* calculate path between start and goal */
    auto [planningSuccessful, pathVector] = dijkstraPlanning.Plan(startPosition, goalPosition);

    /* if collision is (0,0) shortest path does not include a collision point */
    if (collision != std::tuple(0, 0)) {

        /* client path planning needs obstacle as entry to determine first movement direction */

        Node* obstacle;
        //Node firstPos = pathVector.back();
        //bool distX = (std::abs(firstPos.x_ - std::get<0>(collision)) > 1);
        //bool distY = (std::abs(firstPos.y_ - std::get<1>(collision)) > 1);


        /*
        *   if collision at crossroad:
        *   we need 
        */


        //if (distX || distY) {


        //    // Anpassung gemacht bei Startknoten wenn Kollision an Kreuzung ist

        //    /*
        //    *   if distance between collision and first node is higher than 1, collision is on a crossroad.
        //    *   In this case, we need the intermediate node as an obstacle to determine movement direction
        //    */
        //    if (distX) {
        //        unsigned int intermediateX = firstPos.x_ + std::get<0>(collision) / 2;
        //        obstacle = new Node(intermediateX, std::get<1>(collision));
        //    }
        //    else {
        //        unsigned int intermediateY = firstPos.y_ + std::get<1>(collision) / 2;
        //        obstacle = new Node(std::get<0>(collision), intermediateY);
        //    }
        //}
        //else {
        //}

        /* if collision is not on crossroad */
            if (std::get<0>(collision) % 2 == 0 || std::get<1>(collision) == 0) {

                /*  collision occured on a intermediate node and thus can be used for movement direction */
                obstacle = new Node(std::get<0>(collision), std::get<1>(collision));

                /* add obstacle to path list */
                pathVector.push_back(*obstacle);
        }
    }

    /*
    *   pathCoordinatesList is ordered goal to start,
    *   reverse list to have steps from start to goal instead
    *   or in our case from collision to goal
    */
    std::reverse(pathVector.begin(), pathVector.end());

    return pathVector;
}
