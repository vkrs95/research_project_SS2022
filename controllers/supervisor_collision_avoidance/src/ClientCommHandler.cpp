#include "ClientCommHandler.h"

/**********************************************************************
* 
*   ClientCommHandler class functions 
* 
***********************************************************************/


/*   Objects of this class are used to handle the connection between a server socket 
*   and a client robot. In the first place the client is registered with a name and
*   its unique socket port. The registration is acknowledged by the ClientCommHandler.
*   
*   If the robot detects a collision it sends its start-goal nodes and the grid node
*   where the collision has been detected to the ClientCommHandler. Next the supervisor
*   calculates a new path to the configured goal of the robot and sends the information
*   back to the client. 
*/

ClientCommHandler::ClientCommHandler(int clientSocket)
{
    mClientSocket = clientSocket;
    mClientName = "undefined";

    commHandlingThread = new std::thread(&ClientCommHandler::socketCommunicationHandlerRoutine, this);
    commHandlingThread->detach();
}


void ClientCommHandler::socketCommunicationHandlerRoutine(void)
{
    /* start endless thread routine */

    while (true) {
        /* periodically check for message from client */
        receiveMessage();

        /* periodically check for ordered message to send to client */
        sendMessage();

        /* TODO: how long should the thread wait until next check ? */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}


void ClientCommHandler::receiveMessage(void)
{
    char recvBuffer[Message::MAX_MSG_LEN];

    /* check socket if there is something to read */
    fd_set rfds;
    struct timeval tv = { 0, 100 };   // timeval of 100 ms

    FD_ZERO(&rfds);
    FD_SET(mClientSocket, &rfds);

    /*
    *   Check the readability status of the communication socket. Readability means that queued
    *   data is available for reading such that a call to recv is guaranteed not to block.
    */
    int number = select(mClientSocket, &rfds, NULL, NULL, &tv);

    if (number != 0) {
        int recvSize = recv(mClientSocket, recvBuffer, Message::MAX_MSG_LEN, 0);

        if (recvSize > 0) {

            Message* msg = new Message(recvBuffer, recvSize);

            /* might be first message sent by client containing client's name */
            if (mClientName.compare("undefined") == 0) {

                /* check if message is of type REGISTER */
                if (msg->getType() == MessageType::REGISTER) {

                    /* send registration ACK */
                    registerNameAndSendACK(msg);
                    return;
                }
                /* if MessageType != REGISTER just continue adding message to inbox */
            }

            /* add received message to internal inbox */
            std::unique_lock<std::mutex> msgInboxLock(msgMutex);
            mMsgInbox.push_back(*msg);
            msgInboxLock.unlock();

            // std::cout << "ClientCommHandler: received message '" << msg->getMessageAsChar() << "' and pushed in inbox." << std::endl;
        }
    }
}

void ClientCommHandler::sendMessage(void)
{
    Message* lastMessage;
    int sendResult;

    while (!mMsgOutbox.empty()) {

        /* set local mutex and get oldest message from outbox */
        std::unique_lock<std::mutex> msgOutboxLock(msgMutex);
        
        lastMessage = new Message(mMsgOutbox.front());
        mMsgOutbox.pop_front();              // remove the message from list
        
        msgOutboxLock.unlock();

        /* send message to client via tcp socket */
        sendResult = send(mClientSocket, lastMessage->getMessageAsChar(), lastMessage->getSize(), 0);

        if (sendResult == SOCKET_ERROR) {
            std::cerr << "ClientCommHandler: Failed to send message to " << mClientName << std::endl;
        }
        /*else {
            std::cout << "ClientCommHandler: Sent message content '" << lastMessage->getMessageAsChar() << "' to " << mClientName << " with length: " << sendResult << std::endl;
        }*/
    }
}

void ClientCommHandler::registerNameAndSendACK(Message* message)
{
    Message* msg = new Message(*message);

    mClientName = msg->getPayload();
    std::cout << "ClientCommHandler: client '" << mClientName << "' registered on socket " << mClientSocket << "!" << std::endl;
    
    /* send ACK to client */
    std::string ackData = "0;ACK";

    Message* ackMsg = new Message(ackData);
    addMsgToOutbox(ackMsg);
}

void ClientCommHandler::addMsgToOutbox(Message* message)
{
    Message* msg = new Message(*message);

    /* set local mutex and add passed message to outbox */
    std::unique_lock<std::mutex> msgOutboxLock(msgMutex);
    mMsgOutbox.push_back(*msg);
    msgOutboxLock.unlock();
}

Message* ClientCommHandler::getInboxMessage(void)
{
    Message* lastMessage;

    if (mMsgInbox.empty()) {
        lastMessage = nullptr;
    }
    else {
        /* at least one message in inbox, set local mutex and pop next message from inbox */
        std::unique_lock<std::mutex> msgInboxLock(msgMutex);
        
        lastMessage = new Message(mMsgInbox.front());    // get oldest message from FIFO
        mMsgInbox.pop_front();              // remove the message from list

        msgInboxLock.unlock();

        //std::cout << "New message in inbox with content: " << lastMessage->getMessageAsChar() << std::endl;
    }
    return lastMessage;
}

std::string ClientCommHandler::getClientName(void)
{
    return mClientName;
}
