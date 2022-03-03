#include "epuck_autonomous_collision_avoidance.h"
#pragma once

class Pathplanner
{
public:
    std::vector<MovingDirection> path_direction_list;
    std::vector<RobotHeading> heading_list;
    AStar::CoordinateList coordinate_list;

    RobotHeading alternativeHeading;
    bool flag_alternative_planning;

    Pathplanner();
    void PathPlanning(AStar::Vec2i start_pos, AStar::Vec2i goal_pos);
    void PathPlanning(AStar::CoordinateList new_wall, AStar::Vec2i start_pos, AStar::Vec2i goal_pos);
    void AlternativePlanning(AStar::Vec2i new_wall, AStar::Vec2i start_pos, AStar::Vec2i goal_pos, RobotHeading currentheading);
    void GeneratePointCoordinateList();
    void SetMatrixDimension(unsigned int dimension);
    MovingDirection EvaluateDirection(AStar::Vec2i predecessor, AStar::Vec2i current, AStar::Vec2i successor);

    std::vector<AStar::Vec2i> MP_List = {};

private:
    void SetInstuctionList(AStar::CoordinateList path);
    RobotHeading DetermineEpuckInitHeading(unsigned int start_x, unsigned int start_y);
    RobotHeading InverseHeading(RobotHeading currentHeading);
    void AddWallsToWorldGenerator(AStar::Generator* generator);

    int ARENA_NUMBER_OF_LINES_PER_SIDE;
    int MATRIX_N;
};
