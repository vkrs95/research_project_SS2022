#pragma once

#include <vector>
#include <string>

using coordinate = std::tuple<int, int>;

class ICommModule
{

public:
    virtual bool registerAtSupervisor(std::string robotName) = 0;
    virtual void unregisterFromSupervisor(std::string reason = std::string("none")) = 0;
    virtual bool reportCollision(coordinate startXY, 
                                    coordinate goalXY, 
                                    coordinate collisionXY) = 0;
    virtual bool receiveCollisionReply(std::vector<coordinate>* path) = 0;
};
