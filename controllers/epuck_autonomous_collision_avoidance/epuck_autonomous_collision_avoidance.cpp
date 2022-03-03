#include "RobotRoutine.h"
#include "Pathplanner.h"
#include "ObstacleAvoidance.h"
#include "QRModule.h"

int main(int argc, char **argv) {
    // create all Object instances
    Robot* robot = new Robot();
    RobotRoutine* robotroutine = new RobotRoutine(robot);
    Pathplanner* pathplanner = new Pathplanner();
    ObstacleAvoidance* obstacleavoidance = new ObstacleAvoidance;
    QRModule* qrmodule = new QRModule();

    // local variables
    int timeStep = (int)robot->getBasicTimeStep();
    unsigned int path_direction_list_iterator = 0;
    unsigned int crossroad_turn_done = 3500;
    unsigned int turn_around_done = crossroad_turn_done;

    unsigned int led_counter = 1;
    bool performing_turn_on_crossroad = false;
    bool end_of_line_goal_reached = false;
    unsigned int turn_counter = 0;
    unsigned int end_of_line_detection_counter = 0;
    bool init_procedure_done = false;
    bool path_planning_completed = false;

    bool obstacle_detected = false;
    bool crossroad_detected = false;
    bool get_next_direction = true;


    unsigned int path_iterator = 1;

    // pathplanning
    AStar::Vec2i start_position;
    AStar::Vec2i goal_position;
        
    robotroutine->EnableEpuckCam();


    int ground_sensor_jitter = 0;
    bool performing_turn = false;
    bool avoid_second_collision = false;

    MovingDirection currentdirection;

    bool temporary_helper = false;
    AStar::Vec2i pred_post;
    AStar::CoordinateList new_wall;

    unsigned int qr_distance_to_scan_pos = 2000;
    unsigned int init_procedure_distance_to_scan_counter = 0;

    bool epuck_endless_mode = true;

    unsigned int world_dimension = 0;
    unsigned int start_pos_index = 0;
    unsigned int goal_pos_index = 0;


    // Main loop:
    while (robot->step(timeStep) != -1) {

        robotroutine->ReadSensors();

        // LED cycle
        if (end_of_line_goal_reached) 
        {
            robotroutine->AllLightsOnLED();
        }
        else 
        {
            if ((timeStep * led_counter++) >= led_time_step)
            {
                robotroutine->CyclicBlinkingLED();
                led_counter = 1;
            }
        }

        if (!init_procedure_done) {

            if (path_planning_completed) {

                robotroutine->PerformTurnAround();
                turn_counter += timeStep;

                if (turn_counter >= turn_around_done) {
                    turn_counter = 0;
                    init_procedure_done = true;
                }
            }
            else {
                if (init_procedure_distance_to_scan_counter >= qr_distance_to_scan_pos) {

                    robotroutine->PerformHalt();

                    robotroutine->TakeCameraSnapshot();
                    robotroutine->DisableEpuckCam();

                    AStar::Vec2i start_goal_coordinates[2];
                    if (qrmodule->ReadCoordinatesFromQrImage(robotroutine->qr_img_file_name, &start_pos_index, &goal_pos_index, &world_dimension)) {


                        pathplanner->SetMatrixDimension(world_dimension);

                        // save positions read from QR code
                        start_position = pathplanner->MP_List.at(start_pos_index);
                        goal_position = pathplanner->MP_List.at(goal_pos_index);

                        std::cout << "------------------------" << "\n";
                        std::cout << "E-Puck: " << robotroutine->epuck_name << " in " << world_dimension << "x" << world_dimension << " map\n";
                        std::cout << "Start Position: P" << start_pos_index + 1 << "(" << start_position.x << ", " << start_position.y <<
                            ")\nGoal Position: P" << goal_pos_index + 1 << "(" << goal_position.x << ", " << goal_position.y << ")" << "\n";

                    }
                    // check if invalid coordinates have been written into the vectors
                    else {
                        end_of_line_goal_reached = true;
                        // TODO: error handling if image could not be read
                        return -1;
                    }

                    /* path planning */
                    pathplanner->PathPlanning(start_position, goal_position);

                    init_procedure_distance_to_scan_counter = 0;
                    path_planning_completed = true;
                }
                else {
                    robotroutine->LineFollowingModule();
                    init_procedure_distance_to_scan_counter += timeStep;
                }
            }
        }
        else if(!end_of_line_goal_reached) {

            if (robotroutine->DetectLineCrossroad())
            {
                ground_sensor_jitter++;
                if (ground_sensor_jitter > 4 && !performing_turn)
                {
                    robotroutine->AllLightsOnLED();
                    crossroad_detected = true;
                    get_next_direction = true;
                    ground_sensor_jitter = 0;
                }
            }

            // set flag if a crossroad has been detected
            //if (robotroutine->DetectLineCrossroad())
            //{
            //    performing_turn_on_crossroad = true;
            //}

            if (crossroad_detected)
            {
                //if (path_direction_list_iterator < pathplanner->path_direction_list.size())
                //{
                //    MovingDirection next_direction = pathplanner->path_direction_list.at(path_direction_list_iterator);
                //    if (next_direction == turn_left)
                //        robotroutine->OnCrossroadTurnLeft();
                //    else if (next_direction == turn_right)
                //        robotroutine->OnCrossroadTurnRight();
                //    else
                //    {
                //        robotroutine->lfm_speed[LEFT] = robotroutine->LFM_FORWARD_SPEED;
                //        robotroutine->lfm_speed[RIGHT] = robotroutine->LFM_FORWARD_SPEED;
                //    }

                //    turn_counter += timeStep;

                //    if (turn_counter >= crossroad_turn_done)
                //    {
                //        turn_counter = 0;
                //        performing_turn_on_crossroad = false;
                //        path_direction_list_iterator++;
                //    }
                //}

                if (get_next_direction)
                {
                    AStar::Vec2i predecessor;
                    
                    predecessor = pathplanner->coordinate_list.at(path_iterator - 1);

                    AStar::Vec2i current = pathplanner->coordinate_list.at(path_iterator);
                    AStar::Vec2i successor = pathplanner->coordinate_list.at(path_iterator + 1);

                    currentdirection = pathplanner->EvaluateDirection(predecessor, current, successor);

                    /*std::cout << "Epuck:" << "( " << robot->getName() << " )\n";
                    std::cout << "Pathplanning in iteration:" << "( " << path_iterator << " )\n";
                    std::cout << "New direction (0 = Straight, 1 = left, 2 = right)" << "(" << currentdirection << ")\n";
                    std::cout << "Predecessor P" << "(" << predecessor.x << ", " << predecessor.y << ")\n";
                    std::cout << "Current P" << "(" << current.x << ", " << current.y << ")\n";
                    std::cout << "Successor P" << "(" << successor.x << ", " << successor.y << ")\n";
                    std::cout << "---------------------------------------\n";*/

                    get_next_direction = false;
                    performing_turn = true;
                    
                    path_iterator += 2; // because of padding coordinates
                }
                else
                {
                    if (currentdirection == turn_left)
                        robotroutine->OnCrossroadTurnLeft();
                    else if (currentdirection == turn_right)
                        robotroutine->OnCrossroadTurnRight();
                    else
                    {
                        robotroutine->lfm_speed[LEFT] = robotroutine->LFM_FORWARD_SPEED;
                        robotroutine->lfm_speed[RIGHT] = robotroutine->LFM_FORWARD_SPEED;
                    }

                    turn_counter += timeStep;

                    if (turn_counter >= crossroad_turn_done)
                    {
                        turn_counter = 0;
                        crossroad_detected = false;
                        performing_turn = false;
                    }
                }

            }

            else if (obstacle_detected)
            {
                robotroutine->LFM_FORWARD_SPEED = 200;
                robotroutine->OnCrossroadTurnDegree();

                turn_counter += timeStep;

                if (turn_counter >= crossroad_turn_done)
                {
                    new_wall.push_back(pathplanner->coordinate_list[path_iterator]);
                    start_position = pathplanner->coordinate_list[path_iterator - 1];

                    temporary_helper = true;
                    pred_post = pathplanner->coordinate_list[path_iterator];

                    pathplanner->PathPlanning(new_wall, start_position, goal_position);

                    turn_counter = 0;
                    path_iterator = 1;
                    obstacle_detected = false;
                }
            }

            else if (obstacleavoidance->ObstacleDetection(robotroutine->ps_value))
            {
                //if (temporary_helper)
                //{
                //    robotroutine->LFM_FORWARD_SPEED = 0;
                //    robotroutine->lfm_speed[LEFT] = 0;
                //    robotroutine->lfm_speed[RIGHT] = 0;
                //    avoid_second_collision = true;
                //}
                //else
                //{
                    robotroutine->AllLightsOnLED();
                    robotroutine->LFM_FORWARD_SPEED = -200;
                    robotroutine->lfm_speed[LEFT] = -200;
                    robotroutine->lfm_speed[RIGHT] = -200;
                    obstacle_detected = true;
//                }
            }

            else
            {
                if (pathplanner->coordinate_list.size() > 0 && // direction command list has been initialized already
                    path_iterator >= pathplanner->coordinate_list.size()) // all direction commands have been executed
                {
                    if (epuck_endless_mode) {

                        // reset variables -> reset_controller_state();
                        start_position.x = 0; start_position.y = 0;
                        goal_position.x = 0; goal_position.y = 0;
                        path_planning_completed = false;
                        init_procedure_done = false;
                        path_iterator = 1;
                        new_wall.clear();

                        robotroutine->EnableEpuckCam();
                    }
                    else {
                        // if no endless mode move until end of line and stop
                        if (robotroutine->DetectEndOfLine()) {
                            if (ground_sensor_jitter >= 10) {
                                robotroutine->PerformHalt();
                                end_of_line_goal_reached = true;
                                ground_sensor_jitter = 0;
                            }
                            ground_sensor_jitter++;
                        }
                        else {
                            robotroutine->LineFollowingModule();
                        }
                    }
                }

                else
                {
                    robotroutine->LineFollowingModule();
                }
            }
        }

        robotroutine->SetSpeedAndVelocity();
    };

    delete robot;
    return 0;
}
