//
// Created by matthew on 14/09/17.
//

#include "rover_ik/ik_joint_controller.h"

bool rover_ik::IKJointController::init(hardware_interface::EffortJointInterface *hw, ros::NodeHandle &nh_) {
    std::vector<std::string> jnts;
    ROS_INFO_STREAM("Starting it up");
    if (!nh_.getParam("joints", jnts)) {
        ROS_FATAL_STREAM("Failed to get joints.");
        return false;
    }

    std::string tip;
    std::string base;

    if (!nh_.getParam("base_link", base) || !nh_.getParam("tip_link", tip)) {
        ROS_FATAL_STREAM("Please specify both base and tip joints");
        return false;
    }

    ROS_INFO_STREAM("Starting ctrls and solver with " << jnts.size() << "joints");

    ctrls = new rover_ik::JointPositionControl(jnts, nh_, hw);
    ROS_INFO_STREAM("Going");
    solver = new TRAC_IK::TRAC_IK(base, tip);
    ROS_INFO_STREAM("Going");
    if (!solver->getKDLChain(chain)) {
        ROS_FATAL_STREAM("Invalid chain");
        return false;
    }

    solver2 = new KDL::ChainFkSolverPos_recursive(chain);
    ROS_INFO_STREAM("Gone!");
    seed.resize((unsigned int) jnts.size());
    retrievePositions();

    ROS_INFO_STREAM("Start complete");

    sub = nh_.subscribe("target", 10, &IKJointController::posCB, this);

    KDL::Frame origin;
    solver2->JntToCart(this->seed, origin);

        ROS_INFO_STREAM("X " << origin.p.data[0] << " Y " << origin.p.data[1] << " Z " << origin.p.data[2]);

    serv = nh_.advertiseService("request_position", &IKJointController::reqCB, this);

    return true;
}
