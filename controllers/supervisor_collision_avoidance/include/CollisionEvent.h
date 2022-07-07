/* general imports */
#include <string>
#include <vector>
#include <list>
#include <map>
#include <thread>
#include <algorithm>

#include "PathPlanner.h"

#pragma once

using coordinate = std::tuple<int, int>;


class CollisionEvent {
	
protected:
	/*************************************
	* 
	*	Nested class CollisionParticipant
	* 
	**************************************/
	class CollisionParticipant {
	public:
		std::string mName;
		coordinate mStart;
		coordinate mGoal;

		CollisionParticipant(std::string name, coordinate start, coordinate goal);
		std::vector<Node> getPath(void);
		void setPath(std::vector<Node> path);

	private:
		std::vector<Node> path;
	};

public:
	CollisionEvent(coordinate collisionPoint, PathPlanner* planner);
	//coordinate getCollisionID(void);
	void addParticipant(CollisionParticipant newParticipant);
	void addParticipant(std::string name, coordinate start, coordinate goal);
	bool eventResolved(void);
	std::map<std::string, std::vector<Node>> getParticipants(void);

private:
	void resolveEventThreadRoutine(void);
	int determineDTG(CollisionParticipant* participant);

	PathPlanner* planner;

	std::vector<CollisionParticipant> mParticipants;	// internal list to manage participants
	coordinate mCollisionPoint;
	bool mResolved = false;
	std::thread* resolveEventThread;
	static const int MATRIX_DIM = 6;	// arena lines per side * 2
};


