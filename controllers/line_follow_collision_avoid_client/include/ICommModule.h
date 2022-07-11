#pragma once

#include <vector>
#include <string>

using coordinate = std::tuple<int, int>;

class ICommModule
{

public:
    /* registration functions*/
    virtual bool registerAtSupervisor(std::string robotName) = 0;
    virtual void unregisterFromSupervisor(std::string reason = std::string("none")) = 0;
    
    /* path planning functions */
    virtual bool requestPath(coordinate startXY, coordinate goalXY) = 0;
    virtual bool receivePath(std::vector<coordinate>* path) = 0;
    
    /* collision functions */
    virtual bool reportCollision(coordinate startXY, coordinate goalXY, coordinate collisionXY) = 0;
    virtual bool receiveAlternativePath(std::vector<coordinate>* path) = 0;
};
