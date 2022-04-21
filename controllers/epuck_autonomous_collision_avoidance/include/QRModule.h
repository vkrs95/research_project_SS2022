#pragma once
#include <string>

template<typename T>

class QRModule
{
public:

	/*
	*	Virtual function to read a QR code from camera/image/etc.
	*	A file path of the QR code is necessary as well as a type/structure to store the content 
	*	of the encoded QR code.
	*/
	virtual bool readQRCode(std::string qrFilePath, T* qrContent) = 0;

};

