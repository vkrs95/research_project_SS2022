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
        std::cout << "Could not read image" << std::endl;
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

std::vector<std::string> QRModuleEPuckSGD::splitString(std::string str, std::string delimiter)
{
    std::string strToSplit(str);
    std::vector<std::string> splitContent;
    std::string substr;
    size_t pos = 0;

    while ((pos = str.find(delimiter)) != std::string::npos) {
        substr = strToSplit.substr(0, pos);
        splitContent.push_back(substr);
        strToSplit.erase(0, pos + delimiter.length());
    }

    return splitContent;
}

bool QRModuleEPuckSGD::readQRCode(std::string qrFilePath, SGDQRParams* qrContent)
{

    /*** get text content from QR code image ****/
    std::string qrContentString = getContentStringFromQrImage(qrFilePath);

  
    /*** extract the coordinates out of the qr string ****/
    std::vector<std::string> splitContent;
    std::stringstream ss(qrContentString);
        
    /* split string to get start;goal;dimension */
    splitContent = splitString(qrContentString, ";");

    /*** validity check and coordination extraction ****/
    if (splitContent.size() >= 3) {

        std::vector<std::string> splitStart = splitString(splitContent.at(0), ";");
        std::vector<std::string> splitGoal = splitString(splitContent.at(1), ";");

        qrContent->startXY = { std::stoi(splitStart.at(0)), std::stoi(splitStart.at(1)) };
        qrContent->goalXY = { std::stoi(splitStart.at(0)), std::stoi(splitStart.at(1)) };
        qrContent->mapDimension = std::stoi(splitContent.at(2));

    }
    else {
        // invalid !
        std::cout << "Invalid QR code: failed to read start and goal coordinates!" << "\n";
        return false;
    }

    return true;
}
