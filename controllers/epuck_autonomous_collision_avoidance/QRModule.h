#include "epuck_autonomous_collision_avoidance.h"
#pragma once

#include <zbar.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class QRModule
{
public:
	QRModule();
	~QRModule();
	bool ReadCoordinatesFromQrImage(std::string qr_file, unsigned int* start_index, unsigned int* goal_index, unsigned int* map_dimension);

private:
	//std::string robot_name = "unnamed";
};

