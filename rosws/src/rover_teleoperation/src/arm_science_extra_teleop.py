#!/usr/bin/env python
import rospy
import sensor_msgs.msg
import std_msgs.msg

rospy.init_node("arm_extra_teleop")
pub = rospy.Publisher("/arm_science_sensors/command", std_msgs.msg.Int16, queue_size=10)

POS_BUTTON_MAP = {}  # add button: command mappings here


def on_joy(msg):
    for j, i in enumerate(msg.buttons):
        if i and j in POS_BUTTON_MAP:
            pub.publish(data=POS_BUTTON_MAP[j])
            rospy.loginfo("Going to command {}".format(POS_BUTTON_MAP[j]))


sub = rospy.Subscriber("joy", sensor_msgs.msg.Joy, callback=on_joy, queue_size=10)

rospy.spin()
