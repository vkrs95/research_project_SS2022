#include "Message.h"

Message::Message(const char* data, int dataSize)
{
	mMsgSize = dataSize;
	/* forward message content to interpret data */
	parseMessage(data);
}


Message::Message(std::string data)
{
	mMsgSize = data.size();
	/* forward message content to interpret data */
	parseMessage(data.c_str());
}

Message::Message(MessageType type, std::string payload)
{
	mMsgType = type;
	mPayload = payload;
	mErrCode = MessageErrorCode::SUCCESS;
	mMsgSize = std::string(getMessageAsChar()).size();
}

Message::Message(MessageType type, MessageErrorCode errCode, std::string payload)
{
	mMsgType = type;
	mPayload = payload;
	mErrCode = errCode;
	mMsgSize = std::string(getMessageAsChar()).size();
}

Message::Message(MessageType type, int errCode, std::string payload)
{
	mMsgType = type;
	mMsgSize = payload.size();
	mPayload = payload;
	mErrCode = static_cast<MessageErrorCode>(errCode);
}

std::string Message::getPayload()
{
	return mPayload;
}

int Message::getSize(void)
{
	return mMsgSize;
}

MessageType Message::getType(void)
{
	return mMsgType;
}

MessageErrorCode Message::getErrorCode()
{
	return mErrCode;
}

const char* Message::getMessageAsChar(void)
{
	std::stringstream sStrm;

	/* build string containing type and payload, separated with semicolon */
	sStrm << (int) mMsgType << ";" << (int) mErrCode << ";" << mPayload;
	std::string contentStr(sStrm.str());

	/* use vector as a dynamic container to convert string into char */
	std::vector<char> msgVec(contentStr.c_str(), contentStr.c_str() + contentStr.size() + 1);
		
	return msgVec.data();
}

void Message::parseMessage(const char* data)
{
	std::string strToSplit(data);
	std::vector<std::string> splitContent;
	std::string substr;
	size_t pos = 0;

	while ((pos = strToSplit.find(";")) != std::string::npos) {
		substr = strToSplit.substr(0, pos);             // get content of string until position of delimiter 
		splitContent.push_back(substr);                 // add substring to container
		strToSplit.erase(0, pos + 1);  // remove added substring + delimiter from original string 

		if (splitContent.size() == 2)
			break;
	}

	if (splitContent.size() < 2) {
		std::cout << "Error in parsing message: invalid number of delimiters in data" << std::endl;

		/* set message as invalid */
		mMsgType = MessageType::INVALID;
		mErrCode = MessageErrorCode::INVALID;
		mPayload = "-1"; 

		return; // ERROR 
	}

	/* set message type */
	mMsgType = static_cast<MessageType>(std::stoi(splitContent.at(0)));

	/* set error code */
	mErrCode = static_cast<MessageErrorCode>(std::stoi(splitContent.at(1)));

	/* parse message content */
	mPayload = strToSplit; // set rest of string as payload since type and error code are extracted
}