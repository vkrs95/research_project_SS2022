#pragma once

template<typename T, typename U>

class PathPlanningModule
{
public:

    /*
    *   virtual function to find a path between a start and a goal position. 
    *   The passed parameters of type T could be nodes, coordinates or any desired type.
    *   As a result of type U the path itself could be returned. It is also possible to 
    *   use an internal structure for path planning instead of returning a path.
    */
    virtual U findPath(T startPosition, T goalPosition) = 0;

};
