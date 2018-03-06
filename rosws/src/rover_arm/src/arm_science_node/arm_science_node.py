#!/usr/bin/env python
import rospy
from cont_mode import ensure_controllers_are_loaded, set_manual_mode_on_arm
from teleop import Teleop
import sm


def wait_for_services():
    rospy.wait_for_service("/controller_manager/switch_controller", 30)


if __name__ == "__main__":
    rospy.init_node("arm_science_node")
    wait_for_services()
    rospy.loginfo("service found, starting node")
    ensure_controllers_are_loaded()
    rospy.loginfo("done init!")

    set_manual_mode_on_arm()
    teleop = Teleop()
    teleop.daemon = True
    teleop.start()

    rospy.spin()
