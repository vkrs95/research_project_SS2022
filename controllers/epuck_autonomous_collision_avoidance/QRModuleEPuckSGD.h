#pragma once
#include "QRModule.h"

#include <zbar.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
*   This class represents an implementation of the QRModule used by an EPuck robot. 
*   The module expects to read a QR code containing information with the structure: 'start_position:goal_position:world_dimension'
* 
*       - start position: current position of the EPuck as a number between 1 and 4*dimension of world
* 
*       - goal position: randomly encoded goal position as a number between 1 and 4*dimension of world except start position
*       
*       - world dimension: dimension of the world e.g. '3' for a 3x3 world containing 3 * 4 = 12 positions total (4 sides (top, bottom, left, right) with 
*       3 positions on each side)
*      
*   Visualization:
*       - the 'p' represents a start/goal position, '+' are crossroads, '-' and '|' lines connecting all positions as a grid
* 
*       - all positions are numbered starting with the top row from left to right, followed by the bottom row, the left and then the right side
* 
*       - at each start/goal position a QR is placed containing information with the structure described above
*   
* 
*   In the example below the dimension of the world is 3x3 resulting in 3 * 4 = 12 positions total. The QR code placed on positon p1 could 
*   have the content '1:6:3', '1:12:3', '1:3:3', etc.
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
class QRModuleEPuckSGD :
    public QRModule
{
public:
    QRModuleEPuckSGD();
    ~QRModuleEPuckSGD();

    bool readQRCode(std::string qrFilePath, QRParams* qrContent);

private:
    std::string getContentStringFromQrImage(std::string qrFilePath);

};

