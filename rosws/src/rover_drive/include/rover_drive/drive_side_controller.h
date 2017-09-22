//
// Created by matthew on 9/21/17.
//

#ifndef PROJECT_DRIVE_SIDE_CONTROLLER_H
#define PROJECT_DRIVE_SIDE_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>

namespace rover_drive {

    class DriveSideController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        DriveSideController() {

        }

        ~DriveSideController() {

        }


    };

}
#endif //PROJECT_DRIVE_SIDE_CONTROLLER_H
