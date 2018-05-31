//
// Created by matthew on 5/30/18.
//

#ifndef PROJECT_OXYARM_DEV_H
#define PROJECT_OXYARM_DEV_H

#include <cstdint>
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include "../../../simpli2c/include/simpli2c/simpli2c.h"

namespace rover_science {
    /*
     * 0xCC
     * 0x02 0xPP 0xPP p - signed short
     * 0x01 - request oxy
     */
    const int OXY_ARM_ADDRESS = 0x40;
    const int OXY_ARM_BUS     = 1;

    class OxyArm {
    public:
        OxyArm();
        ~OxyArm();

        void writeMotor(int16_t val);
        float readOxy();

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
    private:
        simpli2c::Device dev;
        diagnostic_updater::Updater upd;
    };

}

#endif //PROJECT_OXYARM_DEV_H
