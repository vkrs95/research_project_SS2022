#include "QRModule.h"

// For barcode reader
using namespace zbar;
using namespace cv;


QRModule::QRModule()
{
}

QRModule::~QRModule()
{
}

bool QRModule::ReadCoordinatesFromQrImage(std::string qr_file, unsigned int* start_index, unsigned int* goal_index, unsigned int* map_dimension)
{
    // Create zbar scanner and configure for QR codes
    ImageScanner scanner;
    scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);

    // read passed image and check if it is successfully loaded
    Mat im = imread(qr_file, IMREAD_GRAYSCALE);

    if (im.empty()) {
        printf("Could not read image");
        return false;
    }

    // Wrap image data in a zbar image
    Image image(im.cols, im.rows, "Y800", (uchar*)im.data, im.cols * im.rows);

    // Scan the image for QRCodes
    int n = scanner.scan(image);

    // get content of qr code
    std::string qr_content = image.symbol_begin()->get_data();


    std::vector<int> map_positions;
    std::stringstream ss(qr_content);

    // extract the numbers out of the qr string
    // this should result in an vector with the length of two
    for (int i; ss >> i;) {
        map_positions.push_back(i);
        if (ss.peek() == ':')
            ss.ignore();
    }

    if (map_positions.size() >= 3) {

        // set start and goal position, value-1 since P1 is list index 0
        *start_index = map_positions.at(0) - 1;
        *goal_index = map_positions.at(1) - 1;
        *map_dimension = map_positions.at(2);        

    }
    else {
        // invalid !
        std::cout << "Invalid QR code: failed to read start and goal coordinates!" << "\n";
    }


    // clean up
    image.set_data(NULL, 0);

    return true;
}
