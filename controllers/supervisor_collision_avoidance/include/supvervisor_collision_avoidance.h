/* webots imports */
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Emitter.hpp>
#include <webots/DistanceSensor.hpp>

/* wifi imports */
#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h> /* definition of inet_ntoa */
#include <errno.h>
#include <fcntl.h>
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <stdlib.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

#pragma once

// All the webots classes are defined in the "webots" namespace
using namespace webots;

/*** object pointers ***/
Robot* robot;
Emitter* emitter;


/*** define member variables ***/