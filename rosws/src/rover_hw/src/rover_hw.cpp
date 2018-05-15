//
// Created by matthew on 9/21/17.
//

#include <hardware_interface/robot_hw.h>
#include <rover_drive/hw_impl.h>
#include <rover_arm/arm_hw/arm_hw.h>
#include <controller_manager/controller_manager.h>

namespace rover_hw {

    class RoverHW : public hardware_interface::RobotHW {

    public:

        RoverHW() {

            // ------------------ ADD INITIALIZING CALLS HERE --------------------

            arm_hw.init(this);
            drive_hw.init(this);

            // ------------------ END INITIALIZATION SECTION ---------------------

        }

        void read() {

            // ------------------------ READ SECTION -----------------------------

            drive_hw.read();
            arm_hw.read();

            // ------------------------ END READ SEC -----------------------------

        }

        void write() {

            // ----------------------- WRITE SECTION -----------------------------

            drive_hw.write();
            arm_hw.write();

            // ----------------------- END WRITE SEC -----------------------------

        }


    private:

        // ------------------ ADD ALL NEW HW INTERFACES HERE ------------------

        rover_drive::DriveHW drive_hw;
        rover_arm::ArmHW     arm_hw;

        // ---------------------- END HW IFACE SECTION ------------------------
    };

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "rover_hw");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(2);
    rover_hw::RoverHW rover;
    controller_manager::ControllerManager cm(&rover);

    ros::Time ts = ros::Time::now();
    spinner.start();

    ros::Rate rate(50.0);
    while (ros::ok()) {
        ros::Duration d = ts - ros::Time::now();
        ts = ros::Time::now();
        rover.read();
        cm.update(ts, d);
        rover.write();

        rate.sleep();
    }
}