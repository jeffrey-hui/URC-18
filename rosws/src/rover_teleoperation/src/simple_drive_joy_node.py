#!/usr/bin/env python
import math
import std_msgs.msg
import sensor_msgs.msg
import rospy

LEFT_AXIS = 1
RIGHT_AXIS = 4

rospy.loginfo("Starting drive joy teleoperation node")
rospy.init_node("simple_drive_joy_node")
drive_control_left = rospy.Publisher("/left_wheels_controller/cmd", std_msgs.msg.Float64, queue_size=20)
drive_control_right = rospy.Publisher("/right_wheels_controller/cmd", std_msgs.msg.Float64, queue_size=20)


def ctrl_curve(val):
    return math.copysign(val ** 2, val)


def on_joy_data(msg):
    # type: (sensor_msgs.msg.Joy) -> None

    left = ctrl_curve(msg.axes[LEFT_AXIS])
    right = ctrl_curve(msg.axes[RIGHT_AXIS])
    drive_control_left.publish(left)
    drive_control_right.publish(right)

joy_listener = rospy.Subscriber("joy", sensor_msgs.msg.Joy, queue_size=20, callback=on_joy_data)
rospy.spin()
