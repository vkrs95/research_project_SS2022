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

const char* Message::getMessageAsChar(void)
{
	std::stringstream sStrm;

	/* build string containing type and payload, separated with semicolon */
	sStrm << (int) mMsgType << ";" << mPayload;
	std::string contentStr(sStrm.str());

	/* use vector as a dynamic container to convert string into char */
	std::vector<char> msgVec(contentStr.c_str(), contentStr.c_str() + contentStr.size() + 1);
		
	return msgVec.data();
}

void Message::parseMessage(const char* data)
{
	/* determine and set message type */
	unsigned int type = data[0] - '0';
	mMsgType = determineMessageType(type);

	/* parse message content */
	mPayload = std::string(data).erase(0, 2); // erase 2 character since first one is message type, second is separator
}

MessageType Message::determineMessageType(int typeInt)
{
	MessageType mtype; 

	switch (typeInt)
	{
	case 0:
		mtype = MessageType::REGISTER;
		break;

	case 1:
		mtype = MessageType::UNREGISTER;
		break;

	case 2:
		mtype = MessageType::COLLISION;
		break;

	default:
		mtype = MessageType::INVALID;
		break;
	}

	return mtype;
}
