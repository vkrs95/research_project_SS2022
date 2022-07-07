/* webots imports */
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>
#include <webots/Camera.hpp>

/* module imports */
#include "CommModuleSupervisor.h"
#include "CollisionHandler.h"

#pragma once

// All the webots classes are defined in the "webots" namespace
using namespace webots;

/*** object pointers ***/
Robot* robot;
Emitter* emitter;
//Camera* camera;
CommModuleTCPSocketServer* socketServer;
CollisionHandler* collisionHandler;
PathPlanner* pathPlanner;


/*** define member variables ***/
unsigned int mTimeStep = 0;
unsigned int mWorldDimension = 3;


/*** functions ***/
void	robotActiveWait(int numOfSteps);
