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
        ros::NodeHandle nh("/");
        ROS_INFO_STREAM("Doing init on drive");

        if (!this->device.isDisconnected()) {
            setupDeviceOnConnect();
        }

        hardware_interface::ActuatorStateHandle jsLB("drive_motor_back_left", &pos[0], &vel[0], &eff[0]);
        hardware_interface::ActuatorStateHandle jsRB("drive_motor_back_right", &pos[3], &vel[3], &eff[3]);
        hardware_interface::ActuatorStateHandle jsLF("drive_motor_front_left", &pos[1], &vel[1], &eff[1]);
        hardware_interface::ActuatorStateHandle jsRF("drive_motor_front_right", &pos[4], &vel[4], &eff[4]);
        hardware_interface::ActuatorStateHandle jsLM("drive_motor_center_left", &pos[2], &vel[2], &eff[2]);
        hardware_interface::ActuatorStateHandle jsRM("drive_motor_center_right", &pos[5], &vel[5], &eff[5]);

        hw->get<hardware_interface::ActuatorStateInterface>()->registerHandle(jsLB);
        hw->get<hardware_interface::ActuatorStateInterface>()->registerHandle(jsLF);
        hw->get<hardware_interface::ActuatorStateInterface>()->registerHandle(jsLM);
        hw->get<hardware_interface::ActuatorStateInterface>()->registerHandle(jsRB);
        hw->get<hardware_interface::ActuatorStateInterface>()->registerHandle(jsRF);
        hw->get<hardware_interface::ActuatorStateInterface>()->registerHandle(jsRM);
        
        hardware_interface::ActuatorHandle jLB(hw->get<hardware_interface::ActuatorStateInterface>()->getHandle("drive_motor_back_left"), &cmd[0]);
        hardware_interface::ActuatorHandle jRB(hw->get<hardware_interface::ActuatorStateInterface>()->getHandle("drive_motor_back_right"), &cmd[3]);
        hardware_interface::ActuatorHandle jLF(hw->get<hardware_interface::ActuatorStateInterface>()->getHandle("drive_motor_front_left"), &cmd[1]);
        hardware_interface::ActuatorHandle jRF(hw->get<hardware_interface::ActuatorStateInterface>()->getHandle("drive_motor_front_right"), &cmd[4]);
        hardware_interface::ActuatorHandle jLM(hw->get<hardware_interface::ActuatorStateInterface>()->getHandle("drive_motor_center_left"), &cmd[2]);
        hardware_interface::ActuatorHandle jRM(hw->get<hardware_interface::ActuatorStateInterface>()->getHandle("drive_motor_center_right"), &cmd[5]);

        act_vel_interface.registerHandle(jLB);
        act_vel_interface.registerHandle(jLF);
        act_vel_interface.registerHandle(jLM);
        act_vel_interface.registerHandle(jRB);
        act_vel_interface.registerHandle(jRF);
        act_vel_interface.registerHandle(jRM);

        hw->registerInterface(&act_vel_interface);

        diag_dhd.hardwareID = "due-drive";
        diag_dhd.data["connected"] = !this->device.isDisconnected() ? "yes" : "no";
        diag_dhd.data["device"] = "/dev/i2c-" + std::to_string(ARDUINO_DRIVE_BUS);
        diag_dhd.data["address"] = std::to_string(ARDUINO_DRIVE_ADDRESS);
        controller_diagnostics::DiagnosticHandle dH("drive_arduino", &diag_dhd);
        
        hw->get<controller_diagnostics::DiagnosticStateInterface>()->registerHandle(dH);

        std::string robot_description;
        if (!nh.getParam("robot_description", robot_description)) {
            ROS_FATAL_STREAM("Failed to load robot_description, this node will probably now crash");
        }

        this->transmission_loader_ = new transmission_interface::TransmissionInterfaceLoader(hw, &this->robot_transmissions);
        transmission_interface::TransmissionParser parser;
        std::vector<transmission_interface::TransmissionInfo> infos;
        parser.parse(robot_description, infos);
        for (auto e : infos) {
            if (boost::starts_with(e.name_, "wheel")) {
                if (!this->transmission_loader_->load(e)) {
                    ROS_FATAL_STREAM("ASDF");
                }
            }
        }
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
                    ROS_INFO_STREAM("Connected!");
                }
                else {
                    ROS_WARN_STREAM("Failed to connect to I2C device, retry in 3 seconds");
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
        this->robot_transmissions.get<transmission_interface::JointToActuatorVelocityInterface>()->propagate();
        ROS_INFO_STREAM("cmd[0] " << cmd[0]);
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
