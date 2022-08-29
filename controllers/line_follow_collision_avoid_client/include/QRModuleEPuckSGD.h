#pragma once
#include "IQRModule.h"

#include <iostream>
#include <tuple>

#include <zbar.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
*   This class represents an implementation of the IQRModule used by an EPuck robot. 
*   The module expects to read a QR code containing information with the structure: '(start_x, start_y);(goal_x, goal_y);world_dimension'
* 
*       - start position: current position of the EPuck as x-y-coordinate 
* 
*       - goal position: randomly encoded goal position as x-y-coordinate 
*       
*       - world dimension: dimension of the world e.g. '3' for a 3x3 world containing 3 * 4 = 12 positions total (4 sides (top, bottom, left, right) with 
*       3 positions on each side)
*      
*   Visualization:
*       - the 'p' represents a start/goal position, '+' are crossroads, '-' and '|' lines connecting all positions as a grid
*
*       - at each start/goal position a QR is placed containing information with the structure described above
*   
* 
*   In the example below the dimension of the world is 3x3 resulting in 3 * 4 = 12 positions total. The QR code placed on positon p1 could 
*   have the content '(1, 0);(6, 5);3', '(1, 0);(3, 6);3', etc.
* 
*       p1  p2  p3
         |   |   |
*   p7 - + - + - + - p10
*        |   |   |
*   p8 - + - + - + - p11
*        |   |   |
*   p9 - + - + - + - p12
*        |   |   |
*       p4  p5  p6
* 
*/
class SGDQRParams {
public:
    std::tuple<int, int> startXY;
    std::tuple<int, int> goalXY;
    unsigned int mapDimension;
};

class QRModuleEPuckSGD :
    public IQRModule<SGDQRParams>
{
public:
    QRModuleEPuckSGD();
    ~QRModuleEPuckSGD();

    /*********************************************************
    *
    *	public functions
    *
    *********************************************************/

    bool readQRCode(std::string qrFilePath, SGDQRParams* qrContent) override;

private:

    /*********************************************************
    *
    *	private functions
    *
    *********************************************************/

    std::string getContentStringFromQrImage(std::string qrFilePath);
    std::vector<std::string> splitString(std::string str, std::string delimiter);
    std::vector<std::string> processCoordinateString(std::string str);

};