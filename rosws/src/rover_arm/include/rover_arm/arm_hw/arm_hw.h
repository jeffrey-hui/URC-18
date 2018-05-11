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
    const uint8_t ARDUINO_ARM_ADDRESS = 0x23;

    const int ADDRESS = 0x30;
    const int BUS = 1;

    const int ENCODER_SLIDEPOLE_A = 22; // slider pole (vertical)
    const int ENCODER_SLIDEPOLE_B = 23;

    const int ENCODER_SLIDEUNIT_A = 24; // slider unit to inner arm pole
    const int ENCODER_SLIDEUNIT_B = 25;

    const int ENCODER_INNEROUTR_A = 26; // inner arm to outer arm
    const int ENCODER_INNEROUTR_B = 27;

    const int MOTOR_SLIDEPOLE = 2;
    const int MOTOR_SLIDEUNIT = 3;
    const int MOTOR_INNEROUTR = 4;
    const int MOTOR_GRIPPTILT = 5;

    class ArmHW {
    public:
        ArmHW() : device(ADDRESS, BUS) {};
        ~ArmHW() {
            delete this->transmission_loader_;
        }

        void init(hardware_interface::RobotHW *hw);

        void read();

        void write();

    private:
        union {
            double cmd[4] = {0, 0, 0, 0};
            double eff[4];
        };
        double pos[4] = {0, 0, 0, 0};
        double vel[4] = {0, 0, 0, 0};

        void setupDeviceOnConnect();

        eml_uberdriver::Encoder  jointEncoders[4];
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
