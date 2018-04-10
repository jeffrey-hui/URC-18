//
// Created by matthew on 9/21/17.
//

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <eml_uberdriver/ard_device.h>
#include "constants.h"
#include <rover_drive/hw_impl.h>

namespace rover_drive {
    
    uint16_t convert_to_msecs(double vel) {
        double max_val = static_cast<double>(WHEEL_MAX_RADIANS_PER_SECOND);
        double clamped = std::min(max_val, std::max(-max_val, vel));
        double v = clamped / max_val;
        return (uint16_t)(((MOTOR_OFFSET * v)) + MOTOR_MID);
    }

    void DriveHW::init(hardware_interface::RobotHW *hw) {
        if (!this->device.isDisconnected()) {
            setupDeviceOnConnect();
        }

        hardware_interface::JointStateHandle jsLB("leg_to_wheel_back_left", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle jsRB("leg_to_wheel_back_right", &pos[3], &vel[3], &eff[3]);
        hardware_interface::JointStateHandle jsLF("leg_to_wheel_front_left", &pos[1], &vel[1], &eff[1]);
        hardware_interface::JointStateHandle jsRF("leg_to_wheel_front_right", &pos[4], &vel[4], &eff[4]);
        hardware_interface::JointStateHandle jsLM("leg_to_wheel_center_left", &pos[2], &vel[2], &eff[2]);
        hardware_interface::JointStateHandle jsRM("leg_to_wheel_center_right", &pos[5], &vel[5], &eff[5]);

        jnt_state_interface.registerHandle(jsLB);
        jnt_state_interface.registerHandle(jsLF);
        jnt_state_interface.registerHandle(jsLM);
        jnt_state_interface.registerHandle(jsRB);
        jnt_state_interface.registerHandle(jsRF);
        jnt_state_interface.registerHandle(jsRM);
        
        hw->registerInterface(&jnt_state_interface);
        hardware_interface::JointHandle jLB(jnt_state_interface.getHandle("leg_to_wheel_back_left"), &cmd[0]);
        hardware_interface::JointHandle jRB(jnt_state_interface.getHandle("leg_to_wheel_back_right"), &cmd[3]);
        hardware_interface::JointHandle jLF(jnt_state_interface.getHandle("leg_to_wheel_front_left"), &cmd[1]);
        hardware_interface::JointHandle jRF(jnt_state_interface.getHandle("leg_to_wheel_front_right"), &cmd[4]);
        hardware_interface::JointHandle jLM(jnt_state_interface.getHandle("leg_to_wheel_center_left"), &cmd[2]);
        hardware_interface::JointHandle jRM(jnt_state_interface.getHandle("leg_to_wheel_center_right"), &cmd[5]);

        jnt_vel_interface.registerHandle(jLB);
        jnt_vel_interface.registerHandle(jLF);
        jnt_vel_interface.registerHandle(jLM);
        jnt_vel_interface.registerHandle(jRB);
        jnt_vel_interface.registerHandle(jRF);
        jnt_vel_interface.registerHandle(jRM);

        hw->registerInterface(&jnt_vel_interface);

        diag_dhd.hardwareID = "due-drive";
        diag_dhd.data["connected"] = !this->device.isDisconnected() ? "yes" : "no";
        diag_dhd.data["device"] = "/dev/i2c-" + std::to_string(ARDUINO_DRIVE_BUS);
        diag_dhd.data["address"] = std::to_string(ARDUINO_DRIVE_ADDRESS);
        controller_diagnostics::DiagnosticHandle dH("drive_arduino", &diag_dhd);
        
        diag_interface.registerHandle(dH);

        hw->registerInterface(&diag_interface);
    }

    void DriveHW::read() {
        if (this->device.isDisconnected()) {
            diag_dhd.data["connected"] = "no";
            diag_dhd.status = diagnostic_msgs::DiagnosticStatus::ERROR;
            diag_dhd.message = "The arduino is not connected or is not listening to i2c";

            if ((ros::Time::now() - this->lastReconnectAttemptTime).toSec() > 3.0) {
                this->lastReconnectAttemptTime = ros::Time::now();
                if (this->device.tryOpen()) {
                    this->setupDeviceOnConnect();
                    ROS_WARN_STREAM("The I2C device for drive is not responding, attempting reconnect");
                }
            }
        }
        else {
            diag_dhd.data["connected"] = "yes";
            diag_dhd.status = diagnostic_msgs::DiagnosticStatus::OK;
            diag_dhd.message = "The arduino is connected and responding to i2c";
        }
    }

    void DriveHW::write() {
        device.writeMicroseconds(LEFT_BACK_WHEEL, convert_to_msecs(cmd[0]));
        device.writeMicroseconds(LEFT_FRONT_WHEEL, convert_to_msecs(cmd[1]));
        device.writeMicroseconds(LEFT_MID_WHEEL, convert_to_msecs(cmd[2]));
        device.writeMicroseconds(RIGHT_BACK_WHEEL, convert_to_msecs(cmd[3]));
        device.writeMicroseconds(RIGHT_FRONT_WHEEL, convert_to_msecs(cmd[4]));
        device.writeMicroseconds(RIGHT_MID_WHEEL, convert_to_msecs(cmd[5]));
    }

    void DriveHW::setupDeviceOnConnect() {
        device.openPinAsMotor(LEFT_BACK_WHEEL);
        device.openPinAsMotor(LEFT_MID_WHEEL);
        device.openPinAsMotor(LEFT_FRONT_WHEEL);
        device.openPinAsMotor(RIGHT_BACK_WHEEL);
        device.openPinAsMotor(RIGHT_MID_WHEEL);
        device.openPinAsMotor(RIGHT_FRONT_WHEEL);
    }


}