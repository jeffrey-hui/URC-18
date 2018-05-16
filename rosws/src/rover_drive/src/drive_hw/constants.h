//
// Created by matthew on 22/08/17.
//

#ifndef PROJECT_CONSTANTS_H
#define PROJECT_CONSTANTS_H

#include <cstdint>
#include <array>

namespace rover_drive {

    const int WHEEL_MAX_RADIANS_PER_SECOND = 1960;

    const int LEFT_BACK_WHEEL = 2;
    const int LEFT_FRONT_WHEEL = 3;
    const int LEFT_MID_WHEEL = 4;

    const int RIGHT_BACK_WHEEL = 5;
    const int RIGHT_FRONT_WHEEL = 6;
    const int RIGHT_MID_WHEEL = 7;

    const int MOTOR_MID    = 1500;
    const int MOTOR_OFFSET = 700;

}

#endif //PROJECT_CONSTANTS_H
