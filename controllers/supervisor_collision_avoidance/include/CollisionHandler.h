/* general imports */
#include <string>
#include <vector>
#include <list>
#include <map>

#include "CollisionEvent.h"

#pragma once

class CollisionHandler
{

public:
	CollisionHandler(PathPlanner* planner);
	void registerCollision(std::string name, coordinate start, coordinate goal, coordinate collision);
	bool collisionResolved(std::map<std::string, std::pair<int, std::vector<coordinate>>>* clientPathList);

private:
	void processCollisionEvents(void);
	std::map<coordinate, CollisionEvent*> mCollisionList;

	PathPlanner* mPlanner;
};



