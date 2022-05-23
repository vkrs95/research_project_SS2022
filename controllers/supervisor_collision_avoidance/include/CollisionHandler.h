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
	CollisionHandler(void);
	void registerCollision(std::string name, coordinate start, coordinate goal, coordinate collision);
	bool collisionResolved(std::map<std::string, std::vector<Node>>* clientPathList);

private:
	void processCollisionEvents(void);
	std::map<coordinate, CollisionEvent*> mCollisionList;
};



