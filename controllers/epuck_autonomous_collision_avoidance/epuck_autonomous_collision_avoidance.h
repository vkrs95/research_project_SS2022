#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/LED.hpp>
#include <webots/Camera.hpp>

#include <AStar.hpp>

#include <array>
#include <vector>
#include <stdexcept>

#pragma once

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Global Defines
constexpr auto NO_SIDE = -1;
constexpr auto LEFT = 0;
constexpr auto RIGHT = 1;
constexpr auto WHITE = 0;
constexpr auto BLACK = 1;
constexpr auto TIME_STEP = 32;  // [ms]

// 8 IR proximity sensors
constexpr auto NB_DIST_SENS = 8;
constexpr auto PS_RIGHT_00 = 0;
constexpr auto PS_RIGHT_45 = 1;
constexpr auto PS_RIGHT_90 = 2;
constexpr auto PS_RIGHT_REAR = 3;
constexpr auto PS_LEFT_REAR = 4;
constexpr auto PS_LEFT_90 = 5;
constexpr auto PS_LEFT_45 = 6;
constexpr auto PS_LEFT_00 = 7;

constexpr int PS_OFFSET_SIMULATION[NB_DIST_SENS] = { 300, 300, 300, 300, 300, 300, 300, 300 };
constexpr int PS_OFFSET_REALITY[NB_DIST_SENS] = { 480, 170, 320, 500, 600, 680, 210, 640 };

// 3 IR ground color sensors
constexpr auto NB_GROUND_SENS = 3;
constexpr auto GS_BLACK = 310;
constexpr auto GS_GROUND = 800;
constexpr auto GS_LEFT = 0;
constexpr auto GS_CENTER = 1;
constexpr auto GS_RIGHT = 2;

// 8 LEDs
constexpr auto NB_LEDS = 8;
constexpr unsigned int led_time_step = 320; // [ms]

// Moving directions
enum MovingDirection {
    straight_on = 0,
    turn_left,
    turn_right,
    turn_around
};

enum RobotHeading {
    HEADING_NORTH = 0,
    HEADING_EAST,
    HEADING_SOUTH,
    HEADING_WEST
};