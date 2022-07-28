#include <vector>

#undef max // undef to avoid error in path_planning/utils.hpp due to predefined max macro in windows.h
#include "path_planning/dijkstra.hpp"

#pragma once

class PathPlanner
{
public:
	/* constructors */
	PathPlanner(int dimension);
	std::vector<Node> getShortestPath(std::tuple<int, int> start, std::tuple<int, int> goal, std::tuple<int, int> collision = { 0,0 });
	std::vector<Node> getAlternativePath(std::tuple<int, int> start, std::tuple<int, int> goal, std::tuple<int, int> collision);


private:
	void prepareWorldGrid(void);
	void addObstacle(Node obstacle);
	std::vector<Node> doPathPlanning(std::tuple<int, int> collision);

	int worldDimension = -1;

	Node startPosition, goalPosition;

	/* list of (x,y) coordinates from start to goal position */
	std::vector<Node> pathCoordinatesList;

	std::vector<std::vector<int>> worldGrid; // TODO: must be initialized in setMatrixDimension() with set ARENA_NUMBER_OF_LINES_PER_SIDE
};



