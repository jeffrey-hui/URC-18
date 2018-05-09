//
// Created by matthew on 9/22/17.
//

#ifndef PROJECT_HW_IMPL_H_H
#define PROJECT_HW_IMPL_H_H

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_diagnostics/diagnostic_interface.h>
#include "../../../eml_uberdriver/include/eml_uberdriver/ard_device.h"

namespace rover_drive {

    const uint8_t ARDUINO_DRIVE_BUS = 1;
    const uint8_t ARDUINO_DRIVE_ADDRESS = 0x20;

    class DriveHW {
    public:
        DriveHW() : device(ARDUINO_DRIVE_BUS, ARDUINO_DRIVE_ADDRESS) {};

        void init(hardware_interface::RobotHW *hw);

        void read();

        void write();

    private:
        void setupDeviceOnConnect();

        eml_uberdriver::ARDevice device;
        union {
            double cmd[6];
            double vel[6];
        };
        double eff[6] = {0, 0, 0, 0, 0, 0};
        double pos[6] = {0, 0, 0, 0, 0, 0};

        hardware_interface::JointStateInterface jnt_state_interface;
        hardware_interface::VelocityJointInterface jnt_vel_interface;
        controller_diagnostics::DiagnosticStateInterface diag_interface;

        ros::Time lastReconnectAttemptTime = ros::Time::now();

        controller_diagnostics::DiagnosticHandleData diag_dhd;
    };
}

#endif //PROJECT_HW_IMPL_H_H
