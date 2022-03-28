#include "QRModuleEPuckSGD.h"

// For barcode reader
using namespace zbar;
using namespace cv;


QRModuleEPuckSGD::QRModuleEPuckSGD()
{
}

QRModuleEPuckSGD::~QRModuleEPuckSGD()
{
}

std::string QRModuleEPuckSGD::getContentStringFromQrImage(std::string qrFilePath)
{
    // Create zbar scanner and configure for QR codes
    ImageScanner scanner;
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

    // read passed image and check if it is successfully loaded
    Mat im = imread(qrFilePath, IMREAD_GRAYSCALE);

    if (im.empty()) {
        printf("Could not read image");
        return std::string();               // return empty string 
    }

    // Wrap image data in a zbar image
    Image image(im.cols, im.rows, "Y800", (uchar*)im.data, im.cols * im.rows);

    // Scan the image for QRCodes
    int n = scanner.scan(image);

    // get content of qr code
    std::string qrContent = image.symbol_begin()->get_data();


    /*** clean upand return result ****/
    image.set_data(NULL, 0);

    return qrContent;
}

bool QRModuleEPuckSGD::readQRCode(std::string qrFilePath, unsigned int* startIndex, unsigned int* goalIndex, unsigned int* mapDimension)
{

    /*** get text content from QR code image ****/
    std::string qrContentString = getContentStringFromQrImage(qrFilePath);

  
    /*** extract the numbers out of the qr string ****/
    std::vector<int> mapPositions;
    std::stringstream ss(qrContentString);

    // extract the numbers out of the qr string
    // this should result in an vector with the length of two
    for (int i; ss >> i;) {
        mapPositions.push_back(i);
        if (ss.peek() == ':')
            ss.ignore();
    }

    /*** validity check and index extraction ****/
    if (mapPositions.size() >= 3) {

        // set start and goal position, value-1 since P1 is list index 0
        *startIndex = mapPositions.at(0) - 1;
        *goalIndex = mapPositions.at(1) - 1;
        *mapDimension = mapPositions.at(2);

    }
    else {
        // invalid !
        std::cout << "Invalid QR code: failed to read start and goal coordinates!" << "\n";
        return false;
    }

    return true;
}
