//
// Created by matthew on 10/09/17.
//

#ifndef ROVER_IK_JOINTPOSITIONCONTROL_H
#define ROVER_IK_JOINTPOSITIONCONTROL_H

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>

namespace rover_ik {

    class JointPositionControl {

    public:
        JointPositionControl(std::vector<std::string> names, ros::NodeHandle nh_,
                             hardware_interface::EffortJointInterface *hw);
        void updateSetpoints(std::vector<double> chainValues);
        void update(ros::Duration duration);

        std::vector<double> getPositions();

    private:

        std::vector<control_toolbox::Pid>                   pidLoops;
        std::vector<hardware_interface::JointHandle>        jointHandles;
        realtime_tools::RealtimeBuffer<std::vector<double>> setPoints;
        std::vector<ros::NodeHandle>                        nodeHandles;

    };

}


#endif //ROVER_IK_JOINTPOSITIONCONTROL_H
