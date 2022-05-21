/* general imports */
#include <string>
#include <vector>
#include <list>
#include <map>

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
		std::vector<coordinate> getPath(void);

	protected:
		void setPath(std::vector<coordinate> path);

	private:
		std::vector<coordinate> path;
	};

public:
	CollisionEvent(coordinate collisionPoint);
	//coordinate getCollisionID(void);
	void addParticipant(CollisionParticipant newParticipant);
	void addParticipant(std::string name, coordinate start, coordinate goal);
	bool eventResolved(void);
	std::map<std::string, std::vector<coordinate>> getParticipants(void);

private:
	std::list<CollisionParticipant> mParticipants;	// internal list to manage participants
	coordinate mCollisionPoint;
	bool mResolved = false;
};


