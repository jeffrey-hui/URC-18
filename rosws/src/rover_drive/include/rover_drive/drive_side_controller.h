//
// Created by matthew on 9/21/17.
//

#ifndef PROJECT_DRIVE_SIDE_CONTROLLER_H
#define PROJECT_DRIVE_SIDE_CONTROLLER_H

#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <realtime_tools/realtime_buffer.h>
#include <pluginlib/class_list_macros.h>

namespace rover_drive {

    class DriveSideController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {
    public:
        DriveSideController() {

        }

        ~DriveSideController() {
            sub.shutdown();
        }

        virtual bool init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &nh);
        void update(const ros::Time& time, const ros::Duration& period);

        void velCB(const std_msgs::Float64 &p);

    private:

        ros::Subscriber sub;
        std::vector<hardware_interface::JointHandle> handle;
        realtime_tools::RealtimeBuffer<double> dat;

    };

    PLUGINLIB_EXPORT_CLASS(rover_drive::DriveSideController, controller_interface::ControllerBase);

}
#endif //PROJECT_DRIVE_SIDE_CONTROLLER_H
