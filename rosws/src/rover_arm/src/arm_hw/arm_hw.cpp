//
// Created by matthew on 2/15/18.
//

#include <rover_arm/arm_hw/arm_hw.h>

namespace rover_arm {
    uint16_t convertEffToMotorV(double value) {
        return (uint16_t)value; // todo massive
    }
}

void rover_arm::ArmHW::init(hardware_interface::RobotHW *hw) {
    this->device.openPinAsMotor(MOTOR_INNEROUTR);
    this->device.openPinAsMotor(MOTOR_SLIDEUNIT);
    this->device.openPinAsMotor(MOTOR_SLIDEPOLE);

    this->jointEncoders[0] = this->device.openPinAsEncoder(ENCODER_INNEROUTR_A, ENCODER_INNEROUTR_B);
    this->jointEncoders[1] = this->device.openPinAsEncoder(ENCODER_SLIDEUNIT_A, ENCODER_SLIDEUNIT_B);
    this->jointEncoders[2] = this->device.openPinAsEncoder(ENCODER_SLIDEPOLE_A, ENCODER_SLIDEPOLE_B);

    hardware_interface::JointStateHandle jsh_io("arm_slide_pole_to_arm_slider_unit", &pos[0], &eff[0], &vel[0]);
    hardware_interface::JointStateHandle jsh_su("arm_slider_unit_to_arm_xy_pole", &pos[1], &eff[1], &vel[1]);
    hardware_interface::JointStateHandle jsh_sp("arm_inner_to_arm_outer", &pos[2], &eff[2], &vel[2]);

    this->jnt_state_interface.registerHandle(jsh_io);
    this->jnt_state_interface.registerHandle(jsh_su);
    this->jnt_state_interface.registerHandle(jsh_sp);

    hardware_interface::JointHandle jh_io(jsh_io, &cmd[0]);
    hardware_interface::JointHandle jh_su(jsh_su, &cmd[1]);
    hardware_interface::JointHandle jh_sp(jsh_sp, &cmd[2]);

    this->jnt_eff_interface.registerHandle(jh_io);
    this->jnt_eff_interface.registerHandle(jh_su);
    this->jnt_eff_interface.registerHandle(jh_sp);
}

void rover_arm::ArmHW::write() {
    this->device.writeMicroseconds(MOTOR_INNEROUTR, convertEffToMotorV(cmd[0]));
    this->device.writeMicroseconds(MOTOR_SLIDEUNIT, convertEffToMotorV(cmd[1]));
    this->device.writeMicroseconds(MOTOR_SLIDEPOLE, convertEffToMotorV(cmd[2]));
}

void rover_arm::ArmHW::read() {

}
