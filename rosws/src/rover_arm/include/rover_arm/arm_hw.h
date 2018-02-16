//
// Created by matthew on 2/15/18.
//

#ifndef PROJECT_ARM_HW_H
#define PROJECT_ARM_HW_H

#include <eml_uberdriver/eml_uberdriver.h>

namespace rover_arm {
    const int ADDRESS = 0x30;
    const int BUS = 1;

    class ArmHW {
    public:
        ArmHW() : device(ADDRESS, BUS) {};


    private:

        eml_uberdriver::
        eml_uberdriver::ARDevice device;
    };
}

#endif //PROJECT_ARM_HW_H
