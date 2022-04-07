#include "Pathplanner.h"

Pathplanner::Pathplanner() 
{
    flag_alternative_planning = false;
}

void Pathplanner::PathPlanning(AStar::Vec2i start_pos, AStar::Vec2i goal_pos) 
{
    AStar::Generator* generator = new AStar::Generator;

    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1 });
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    AddWallsToWorldGenerator(generator);

    coordinate_list = generator->findPath(goal_pos, start_pos);

    //// debug print
    //for (int i = 0; i < coordinate_list.size(); i++) {
    //    std::cout << "(" << coordinate_list.at(i).x << ", " << coordinate_list.at(i).y << "); ";
    //}
    //std::cout << "\n";
}

void Pathplanner::PathPlanning(AStar::CoordinateList new_wall, AStar::Vec2i start_pos, AStar::Vec2i goal_pos)
{
    coordinate_list.clear();
    AStar::Generator* generator = new AStar::Generator;

    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1});
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    for (int i = 0; i < new_wall.size(); i++)
    {
        generator->addCollision(new_wall.at(i));
    }

    AddWallsToWorldGenerator(generator); // add default walls

    coordinate_list = generator->findPath(goal_pos, start_pos);
}



void Pathplanner::GeneratePointCoordinateList()
{
    int i;

    // top point row
    for (i = 1; i < MATRIX_N; i += 2) {
        MP_List.push_back({ i, 0 });
    }

    // bottom point row
    for (i = 1; i < MATRIX_N; i += 2) {
        MP_List.push_back({ i, MATRIX_N });
    }

    // left side points
    for (i = 1; i < MATRIX_N; i += 2) {
        MP_List.push_back({ 0, i });
    }

    // right side points
    for (i = 1; i < MATRIX_N; i += 2) {
        MP_List.push_back({ MATRIX_N, i });
    }

}

void Pathplanner::AlternativePlanning(AStar::Vec2i new_wall, AStar::Vec2i start_pos, AStar::Vec2i goal_pos, RobotHeading currentheading)
{
    AStar::Generator* generator = new AStar::Generator;

    generator->setWorldSize({ MATRIX_N + 1, MATRIX_N + 1 });
    generator->setHeuristic(AStar::Heuristic::euclidean);
    generator->setDiagonalMovement(false);

    AddWallsToWorldGenerator(generator);
    generator->addCollision(new_wall);

    alternativeHeading = InverseHeading(currentheading);
    flag_alternative_planning = true;

    coordinate_list.clear();
    coordinate_list = generator->findPath(goal_pos, start_pos);
    SetInstuctionList(coordinate_list);
}


void Pathplanner::SetInstuctionList(AStar::CoordinateList path)
{
    int diff_x, diff_y;
    MovingDirection next_direction;
    RobotHeading next_heading;
    RobotHeading epuck_current_heading;

    if (flag_alternative_planning)
    {
        epuck_current_heading = alternativeHeading;
        flag_alternative_planning = false;

        heading_list.clear();
        path_direction_list.clear();
    }
    else
        epuck_current_heading = DetermineEpuckInitHeading(path.at(0).x, path.at(0).y);


    auto first_pos = path.at(1);
    unsigned int last_pos_x = first_pos.x;
    unsigned int last_pos_y = first_pos.y;

    for (int i = 2; i < path.size(); i++) {
        diff_x = path.at(i).x - last_pos_x;
        diff_y = path.at(i).y - last_pos_y;

        if (diff_x != 0) 
        {
            switch (epuck_current_heading)
            {
            case HEADING_NORTH:
                next_direction = diff_x == 1 ? turn_right : turn_left;
                next_heading = HEADING_NORTH;
                break;
            case HEADING_EAST:
                next_direction = diff_x == 1 ? straight_on : turn_around;
                next_heading = HEADING_EAST;
                break;
            case HEADING_SOUTH:
                next_direction = diff_x == 1 ? turn_left : turn_right;
                next_heading = HEADING_SOUTH;
                break;
            case HEADING_WEST:
                next_direction = diff_x == 1 ? turn_around : straight_on;
                next_heading = HEADING_WEST;
                break;
            default:
                std::cout << "Couldnt determine Heading " << "\n";
                throw std::logic_error("Programm should not reach this state: unknown RobotHeading state reached");
                break;
            }
            epuck_current_heading = diff_x == 1 ? HEADING_EAST : HEADING_WEST;
        }
                
        else if (diff_y != 0) 
        {
            switch (epuck_current_heading)
            {
            case HEADING_NORTH:
                next_direction = diff_y == 1 ? turn_around : straight_on;
                next_heading = HEADING_NORTH;
                break;
            case HEADING_EAST:
                next_direction = diff_y == 1 ? turn_right : turn_left;
                next_heading = HEADING_EAST;
                break;
            case HEADING_SOUTH:
                next_direction = diff_y == 1 ? straight_on : turn_around;
                next_heading = HEADING_SOUTH;
                break;
            case HEADING_WEST:
                next_direction = diff_y == 1 ? turn_left : turn_right;
                next_heading = HEADING_WEST;
                break;
            default:
                std::cout << "Couldnt determine Heading " << "\n";
                throw std::logic_error("Programm should not reach this state: unknown RobotHeading state reached");
                break;
            }
            epuck_current_heading = diff_y == 1 ? HEADING_SOUTH : HEADING_NORTH;
        }

        else {
            throw std::logic_error("Programm should not reach this state: neither x nor y has changed");
        }

        if (next_direction == turn_around)
            printf(">>>> Next direction shall be turn_around.");

        if (last_pos_x % 2 != 0 && last_pos_y % 2 != 0) 
        {
            path_direction_list.push_back(next_direction);
            heading_list.push_back(next_heading);
        }

        last_pos_x = path.at(i).x;
        last_pos_y = path.at(i).y;
    }
}


RobotHeading Pathplanner::DetermineEpuckInitHeading(unsigned int start_x, unsigned int start_y) 
{
    RobotHeading epuck_heading;

    if (start_y == 0) {
        epuck_heading = HEADING_SOUTH;
    }
    else if (start_y == MATRIX_N) {
        epuck_heading = HEADING_NORTH;
    }
    else if (start_x == 0) {
        epuck_heading = HEADING_EAST;
    }
    else {
        epuck_heading = HEADING_WEST;
    }

    return epuck_heading;
}

RobotHeading Pathplanner::InverseHeading(RobotHeading currentHeading)
{
    RobotHeading epuck_heading;

    if (currentHeading == HEADING_NORTH) 
    {
        epuck_heading = HEADING_SOUTH;
    }
    else if (currentHeading == HEADING_SOUTH)
    {
        epuck_heading = HEADING_NORTH;
    }
    else if (currentHeading == HEADING_WEST)
    {
        epuck_heading = HEADING_EAST;
    }
    else if (currentHeading == HEADING_EAST)
    {
        epuck_heading = HEADING_WEST;
    }

    return epuck_heading;
}


void Pathplanner::AddWallsToWorldGenerator(AStar::Generator* generator) 
{
    int i, j;

    for (i = 0; i < MATRIX_N + 1; i++) {
        if (i % 2 == 0) {
            for (j = 0; j < MATRIX_N + 1; j++) {
                if (j % 2 == 0) {
                    generator->addCollision({ j,i });
                }
            }
        }
    }
}

void Pathplanner::SetMatrixDimension(unsigned int dimension) 
{
    ARENA_NUMBER_OF_LINES_PER_SIDE = dimension;
    MATRIX_N = ARENA_NUMBER_OF_LINES_PER_SIDE * 2;

    // dynamically initialize list of coordinates 
    GeneratePointCoordinateList();
}


MovingDirection Pathplanner::getNextMovingDirection(size_t pathIterator)// AStar::Vec2i predecessor, AStar::Vec2i current, AStar::Vec2i successor)
{
    MovingDirection nextDirection;

    AStar::Vec2i predecessor = coordinate_list.at(pathIterator - 1);
    AStar::Vec2i current = coordinate_list.at(pathIterator);
    AStar::Vec2i successor = coordinate_list.at(pathIterator + 1);

    predecessor.x = predecessor.x - current.x;
    predecessor.y = predecessor.y - current.y;

    successor.x = successor.x - current.x;
    successor.y = successor.y - current.y;

    int sum_x = predecessor.x + successor.x;
    int sum_y = predecessor.y + successor.y;

    if (sum_x == 0 && sum_y == 0)
    {
        nextDirection = MovingDirection(straight_on);
    }
    else
    {
        int vector_sum_xy = (sum_x + sum_y) * 0.5;
        int pred_suc_sum = predecessor.y + successor.x + successor.y;

        if (vector_sum_xy - pred_suc_sum == 0)
        {
            nextDirection = MovingDirection(turn_left);
        }
        else
        {
            nextDirection = MovingDirection(turn_right);
        }
    }

    /* debug output for node coordinates */
    //std::cout << "Pathplanning in iteration:" << "( " << pathIterator << " )\n";
    //std::cout << "New direction (0 = Straight, 1 = left, 2 = right)" << "(" << nextDirection << ")\n";
    //std::cout << "Predecessor P" << "(" << predecessor.x << ", " << predecessor.y << ")\n";
    //std::cout << "Current P" << "(" << current.x << ", " << current.y << ")\n";
    //std::cout << "Successor P" << "(" << successor.x << ", " << successor.y << ")\n";
    //std::cout << "---------------------------------------\n";

    return nextDirection;
}