//
// Created by matthew on 2/15/18.
//

#ifndef PROJECT_ARM_HW_H
#define PROJECT_ARM_HW_H

#include <eml_uberdriver/eml_uberdriver.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/actuator_command_interface.h>
#include <controller_diagnostics/diagnostic_interface.h>
#include <transmission_interface/transmission_interface_loader.h>

namespace rover_arm {
    const uint8_t ARDUINO_ARM_BUS = 1;
    const uint8_t ARDUINO_ARM_ADDRESS = 0x21;

    //const int ADDRESS = 0x30;
    //const int BUS = 1;

    const int ENCODER_SLIDEPOLE_A = 28; // slider pole (vertical)
    const int ENCODER_SLIDEPOLE_B = 29;

    const int ENCODER_SLIDEUNIT_A = 26; // slider unit to inner arm pole
    const int ENCODER_SLIDEUNIT_B = 27;

    const int ENCODER_INNEROUTR_A = 24; // inner arm to outer arm
    const int ENCODER_INNEROUTR_B = 25;

    const int ENCODER_GRIPPTILT_A = 22;
    const int ENCODER_GRIPPTILT_B = 23;

    const int ENCODER_GRIPPSPIN_A = 30;
    const int ENCODER_GRIPPSPIN_B = 31;
    
    const int ENCODER_GRIPPPPER_A = 32;
    const int ENCODER_GRIPPPPER_B = 33;

    const int MOTOR_SLIDEPOLE = 2;
    const int MOTOR_SLIDEUNIT = 4;
    const int MOTOR_INNEROUTR = 3;
    const int MOTOR_GRIPPTILT = 5;
    const int MOTOR_GRIPPSPIN = 6;
    const int MOTOR_GRIPPPPER = 7;

    class ArmHW {
    public:
        ArmHW() : device(ARDUINO_ARM_BUS, ARDUINO_ARM_ADDRESS) {};
        ~ArmHW() {
            delete this->transmission_loader_;
        }

        void init(hardware_interface::RobotHW *hw);

        void read();

        void write();

    private:
        union {
            double cmd[6] = {0, 0, 0, 0, 0, 0};
            double eff[6];
        };
        double pos[6] = {0, 0, 0, 0, 0, 0};
        double vel[6] = {0, 0, 0, 0, 0, 0};

        double gripper_alt_cmd;

        void setupDeviceOnConnect();

        eml_uberdriver::Encoder  jointEncoders[6];
        eml_uberdriver::ARDevice device;

        hardware_interface::ActuatorStateInterface  act_state_interface;
        hardware_interface::EffortActuatorInterface act_eff_interface;

        controller_diagnostics::DiagnosticStateInterface diag_interface;
        controller_diagnostics::DiagnosticHandleData diag_dhd;
        ros::Time lastReconnectAttemptTime;

        transmission_interface::TransmissionInterfaceLoader *transmission_loader_;
        transmission_interface::RobotTransmissions robot_transmissions;
    };
}

#endif //PROJECT_ARM_HW_H
