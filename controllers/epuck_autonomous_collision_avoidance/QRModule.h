#include "epuck_autonomous_collision_avoidance.h"
#pragma once


class QRModule
{
public:
	/*
	*	virtual function header to read a QR code from camera/image/etc.
	*/
	//virtual void readQRCode() = 0;
	
	/*
	*	extension of readQRCode where the QR code must include a start and goal 
	*	position as well as the dimension of the environment as a single digit
	*/
	virtual bool readQRCode(std::string qrFilePath, unsigned int* startIndex, unsigned int* goalIndex, unsigned int* mapDimension) = 0;

};

