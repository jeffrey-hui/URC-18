//
// Created by matthew on 9/21/17.
//

#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <rover_drive/drive_side_controller.h>

bool rover_drive::DriveSideController::init(hardware_interface::VelocityJointInterface *hw, ros::NodeHandle &nh) {
    sub = nh.subscribe("cmd", 10, &DriveSideController::velCB, this);

    std::vector<std::string> joints;
    if (!nh.getParam("joints", joints)) {
        ROS_FATAL_STREAM("Failed to get joints");
        return false;
    }

    for (auto it : joints) {
        handle.push_back(hw->getHandle(it));
    }

    ROS_INFO_STREAM("Started drive side controller");
    return true;
}

void rover_drive::DriveSideController::update(const ros::Time &time, const ros::Duration &period) {
    double d = *this->dat.readFromRT();
    for (auto it : this->handle) {
        it.setCommand(d);
    }
}

void rover_drive::DriveSideController::velCB(const std_msgs::Float64 &p) {
    this->dat.writeFromNonRT(p.data);
}

PLUGINLIB_EXPORT_CLASS(rover_drive::DriveSideController, controller_interface::ControllerBase);

