/* webots imports */
#include <webots/Robot.hpp>
#include <webots/Emitter.hpp>

/* module imports */
#include "CommModuleSupervisor.h"

#pragma once

// All the webots classes are defined in the "webots" namespace
using namespace webots;

/*** object pointers ***/
Robot* robot;
Emitter* emitter;
CommModuleTCPSocketServer* socketServer;


/*** define member variables ***/
unsigned int mTimeStep = 0;


/*** functions ***/
void	robotActiveWait(int numOfSteps);
