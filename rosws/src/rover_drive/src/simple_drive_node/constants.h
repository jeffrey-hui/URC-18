//
// Created by matthew on 22/08/17.
//

#ifndef PROJECT_CONSTANTS_H
#define PROJECT_CONSTANTS_H

#include <cstdint>
#include <array>

namespace rover_drive {

    const std::array<uint8_t, 1> LEFT_WHEELS = {9}; // expandable for different numbers of wheels
    const std::array<uint8_t, 1> RIGHT_WHEELS = {10};

    const int MOTOR_MID = 1500;
    const int MOTOR_OFFSET = 700;

}

#endif //PROJECT_CONSTANTS_H
