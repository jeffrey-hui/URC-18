//
// Created by matthew on 9/22/17.
//

#ifndef PROJECT_HW_IMPL_H_H
#define PROJECT_HW_IMPL_H_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include "../../src/drive_hw/ard_device.h"

namespace rover_drive {

    class DriveHW {
    public:
        DriveHW() : device(1, 0x30) {};

        void init(hardware_interface::RobotHW *hw);

        void read();

        void write();

    private:
        ARDevice device;
        union {
            double cmd[6];
            double vel[6];
        };
        double eff[6] = {0, 0, 0, 0, 0, 0};
        double pos[6] = {0, 0, 0, 0, 0, 0};

        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;

    };
}

#endif //PROJECT_HW_IMPL_H_H
