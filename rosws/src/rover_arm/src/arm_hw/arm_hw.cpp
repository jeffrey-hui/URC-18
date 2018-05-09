//
// Created by matthew on 2/15/18.
//

#include <rover_arm/arm_hw/arm_hw.h>
#include <controller_diagnostics/diagnostic_interface.h>

namespace rover_arm {
    uint16_t convertEffToMotorV(double value) {
        return (uint16_t)value; // todo massive
    }
}

void rover_arm::ArmHW::init(hardware_interface::RobotHW *hw) {
    if (!this->device.isDisconnected()) {
        this->setupDeviceOnConnect();
    }

    this->lastReconnectAttemptTime = ros::Time::now();

    hardware_interface::JointStateHandle jsh_io("arm_slide_pole_to_arm_slider_unit", &pos[0], &eff[0], &vel[0]);
    hardware_interface::JointStateHandle jsh_su("arm_slider_unit_to_arm_xy_pole", &pos[1], &eff[1], &vel[1]);
    hardware_interface::JointStateHandle jsh_sp("arm_inner_to_arm_outer", &pos[2], &eff[2], &vel[2]);
    hardware_interface::JointStateHandle jsh_tl("arm_outer_to_grip_spin", &pos[3], &eff[3], &vel[3]);

    this->jnt_state_interface.registerHandle(jsh_io);
    this->jnt_state_interface.registerHandle(jsh_su);
    this->jnt_state_interface.registerHandle(jsh_sp);
    this->jnt_state_interface.registerHandle(jsh_tl);

    hardware_interface::JointHandle jh_io(jsh_io, &cmd[0]);
    hardware_interface::JointHandle jh_su(jsh_su, &cmd[1]);
    hardware_interface::JointHandle jh_sp(jsh_sp, &cmd[2]);
    hardware_interface::JointHandle jh_tl(jsh_tl, &cmd[3]);

    this->jnt_eff_interface.registerHandle(jh_io);
    this->jnt_eff_interface.registerHandle(jh_su);
    this->jnt_eff_interface.registerHandle(jh_sp);
    this->jnt_eff_interface.registerHandle(jh_tl);

    diag_dhd.hardwareID = "due-arm";
    diag_dhd.data["connected"] = !this->device.isDisconnected() ? "yes" : "no";
    diag_dhd.data["device"] = "/dev/i2c-" + std::to_string(ARDUINO_ARM_BUS);
    diag_dhd.data["address"] = std::to_string(ARDUINO_ARM_ADDRESS);
    controller_diagnostics::DiagnosticHandle dH("arm_arduino", &diag_dhd);

    diag_interface.registerHandle(dH);

    hw->registerInterface(&diag_interface);
    hw->registerInterface(&jnt_eff_interface);
    hw->registerInterface(&jnt_state_interface);
}

void rover_arm::ArmHW::write() {
    if (!this->device.isDisconnected()) {
        this->device.writeMicroseconds(MOTOR_INNEROUTR, convertEffToMotorV(cmd[0]));
        this->device.writeMicroseconds(MOTOR_SLIDEUNIT, convertEffToMotorV(cmd[1]));
        this->device.writeMicroseconds(MOTOR_SLIDEPOLE, convertEffToMotorV(cmd[2]));
        this->device.writeMicroseconds(MOTOR_GRIPPTILT, convertEffToMotorV(cmd[3]));
    }
}

void rover_arm::ArmHW::read() {
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

        // todo: add encoder reads
    }
}

void rover_arm::ArmHW::setupDeviceOnConnect() {
    this->device.openPinAsMotor(MOTOR_INNEROUTR);
    this->device.openPinAsMotor(MOTOR_SLIDEUNIT);
    this->device.openPinAsMotor(MOTOR_SLIDEPOLE);
    this->device.openPinAsMotor(MOTOR_GRIPPTILT);

    //this->jointEncoders[0] = this->device.openPinAsEncoder(ENCODER_INNEROUTR_A, ENCODER_INNEROUTR_B);
    //this->jointEncoders[1] = this->device.openPinAsEncoder(ENCODER_SLIDEUNIT_A, ENCODER_SLIDEUNIT_B);
    //this->jointEncoders[2] = this->device.openPinAsEncoder(ENCODER_SLIDEPOLE_A, ENCODER_SLIDEPOLE_B);
}
