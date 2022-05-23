#include "PathPlanner.h"

PathPlanner::PathPlanner(int dimension)
{
    worldDimension = dimension;
}

std::vector<Node> PathPlanner::getShortestPath(std::tuple<int, int> start, std::tuple<int, int> goal)
{
    startPosition.x_ == std::get<0>(start); startPosition.y_ == std::get<1>(start);
    goalPosition.x_ == std::get<0>(start); goalPosition.y_ == std::get<1>(start);

    prepareWorldGrid();
    
    Dijkstra dijkstraPlanning(worldGrid);

    /* calculate path between start and goal */
    //auto [planningSuccessful, pathVector] = aStarPlanning.Plan(this->startPosition, this->goalPosition);
    auto [planningSuccessful, pathVector] = dijkstraPlanning.Plan(this->startPosition, this->goalPosition);

    /*
    *   pathCoordinatesList is ordered goal to start,
    *   reverse list to have steps from start to goal instead
    */
    std::reverse(pathVector.begin(), pathVector.end());

    return pathVector;
}

std::vector<Node> PathPlanner::getAlternativePath(std::tuple<int, int> start, std::tuple<int, int> goal, std::tuple<int, int> collision)
{
    startPosition.x_ == std::get<0>(start); startPosition.y_ == std::get<1>(start);
    goalPosition.x_ == std::get<0>(start); goalPosition.y_ == std::get<1>(start);

    prepareWorldGrid();
    
    /* add obstacle to circumnavigate other robot */
    Node obstacle(std::get<0>(start), std::get<1>(start));
    addObstacle(obstacle);

    Dijkstra dijkstraPlanning(worldGrid);

    /* calculate path between start and goal */
    //auto [planningSuccessful, pathVector] = aStarPlanning.Plan(this->startPosition, this->goalPosition);
    auto [planningSuccessful, pathVector] = dijkstraPlanning.Plan(this->startPosition, this->goalPosition);

    /*
    *   pathCoordinatesList is ordered goal to start,
    *   reverse list to have steps from start to goal instead
    */
    std::reverse(pathVector.begin(), pathVector.end());

    return pathVector;
}


void PathPlanner::prepareWorldGrid(void)
{
    int n = worldDimension + 1;

    /* reset world grid */
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
