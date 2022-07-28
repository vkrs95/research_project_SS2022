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

enum class EventState 
{
	UNINITIATED = -2,
	IN_PROGRESS = -1,
	RESOLVED, 
	CANCELED,
	INVALID = 99
};

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
	EventState getState(void);

private:
	void resolveEventThreadRoutine(void);
	int determineDTG(CollisionParticipant* participant);

	PathPlanner* planner;

	std::vector<CollisionParticipant> mParticipants;	// internal list to manage participants
	coordinate mCollisionPoint;
	EventState mState = EventState::UNINITIATED;
	std::thread* resolveEventThread;
	static const int MATRIX_DIM = 6;	// arena lines per side * 2
	static const int PARTICIPANT_TIMEOUT = 2000; // timeout in ms 
	static const int PARTICIPANT_WAIT_TIME = 200; // time duration to sleep in ms 
};


